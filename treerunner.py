"""Common Treerunner APIs"""

import inspect
import logging
import os
import platform
import posixpath
import re
import threading
import time
import common_lib.debug_port_detector

from common_lib.ecu import QnxEcu
from common_lib.serial_comm import SerialComm
from common_lib.sain_smart_relay import SainSmartRelay
from common_lib.debug_port_detector import get_port_name, \
    get_port_baud_rate, PORT_CFG, list_ports
from serial import SerialException
from treerunner_lib.conf import config


# Monkey patching for get_ftdi_ports and relay_board_setup for
# Treerunner specific needs

def get_ftdi_ports():
    """Retrieves a list of FTDI ports.

    :return: A list containing the device name of all FTDI ports
    """
    ftdi_vid = 0x0403
    ftdi_pid = 0x6001

    # Get a list of FTDI Com Ports based on their VID/PID
    ftdi_ports = []
    com_ports = list_ports.comports()

    for port in com_ports:
        if port.vid == ftdi_vid and port.pid == ftdi_pid:
            ftdi_ports.append(port.device)
    return ftdi_ports


def relay_board_setup(port_id_to_detect, ss_pid):
    """Sets up whatever is necessary for port detection.

    :param port_id_to_detect: The port to detect.
    :param ss_pid: The SainSmart Port ID for the Relay Board

    :return:
    """
    ss_jumper_usb_s = 2
    # For specific ports, we want to use the Sain Smart Relay Library
    # that's because these ports require jumpers and power cycles to
    # detect.
    if port_id_to_detect == 'PORT_ID_TREERUNNER':
        # Initialize the relay board.
        sain_smart_relay = SainSmartRelay(ss_pid)
        # Obtain the current relay states.
        sain_smart_relay_states = []
        for i in range(1, 5):
            sain_smart_relay_states.append(
                sain_smart_relay.get_relay_state(i))
        # Unset all the relays.
        if any(sain_smart_relay_states):
            sain_smart_relay.unset_all_relays()
            time.sleep(0.5)
        # Enable USB_S jumper.
        sain_smart_relay.set_relay_state(ss_jumper_usb_s, True)
        time.sleep(10)  # Wait a bit for Windows to enumerate all ports.
        return sain_smart_relay, sain_smart_relay_states
    return None, None


common_lib.debug_port_detector.get_ftdi_ports = get_ftdi_ports
common_lib.debug_port_detector._setup = relay_board_setup


class Treerunner(QnxEcu):
    """Utilities to interact with the Treerunner board"""

    def __init__(self, ssh_client, serial_port=None):
        """
        Create Treerunner instance

        :param ssh_client: SHClient to connect to Treerunner target
        :param serial_port: port to use for serial communication
        """
        self.serial_port = serial_port
        self.logger = logging.getLogger('treerunner')
        #self.start_sshd_server()
        self.ssh_client = ssh_client

    def get_log_tag(self):
        """
        Helper method to get a prefix for log statements

        :param self: a class object
        :return: string prefix for log statements, made up of the class
            name and the executing function name.
        """
        return "{}::{}- ".format(self.__class__.__name__,
                                 str(inspect.stack()[1][3]))

    def start_sshd_server(self):
        """
        Start sshd server daemon via serial
        """
        log_tag = self.get_log_tag()
        assert self.setup_serial_port(), "{} Failure to setup serial " \
                                         "port".format(log_tag)

        self.logger.info("{} Attempting to start sshd server "
                         "daemon".format(log_tag))
        try:
            if not self._is_sshd_server_running():
                if not self.serial_port.is_open():
                    self.logger.info(
                        "{} serial is not open. Proceeding to open it "
                        "".format(log_tag))
                    self.serial_port.open()
                    time.sleep(1)
                self.serial_port.write("/usr/sbin/sshd")
                # Little timeout to ensure the process starts
                time.sleep(1)
                self.serial_port.write('/proc/boot/pidin | grep sshd')
                assert self._get_pid_from_serial(self.serial_port), \
                    "Failed to verify if the process started or not"
                self.serial_port.close()
                self.logger.info(
                    "{} Successfully started the sshd server "
                    "daemon".format(log_tag))
        except SerialException as e:
            self.logger.error("{} Failed to write to serial "
                              "port: {}".format(log_tag, str(e)))

    def _is_sshd_server_running(self, timeout=1):
        """
        Check if the sshd server daemon is running by attempting a
        ssh connection

        :param timeout: timeout
        :return:
        """
        try:
            self.ssh_client.connect(timeout=timeout)
            self.ssh_client.close()
            return True
        except Exception:
            return False

    def _get_pid_from_serial(self, serial, timeout=20):
        """
        Get pid from the serial buffer

        :param serial: serial connection
        :param timeout: timeout for the timer
        :return: True if pidin found, false otherwise
        """
        log_tag = self.get_log_tag()
        self.logger.info("{} Verifying the process was "
                         "started".format(log_tag))
        pidin_regex = r'(.*?)\d\susr/sbin'
        success = False
        timer = threading.Timer(timeout, lambda: None)
        timer.start()
        while timer.is_alive():
            response = serial.ser.readline().decode("utf-8")
            pidin = re.search(pidin_regex, response)
            if pidin and "usr/sbin/sshd" in response:
                self.logger.info("{} process started as: {}".format(
                    log_tag, response))
                timer.cancel()
                success = True
                break
        return success

    def setup_serial_port(self, port_id='PORT_ID_TREERUNNER'):
        """
        Setup serial port. If the port is not specified, it will be
        set dynamically.

        :param port_id: id of the port to look for
        :return: True if successful, False otherwise
        """
        log_tag = self.get_log_tag()
        if self.serial_port is not None and \
           isinstance(self.serial_port, str):
            self.logger.info(
                "{} Serial port specified '{}' - "
                "Creating instance".format(log_tag, self.serial_port))
            port = SerialComm(self.serial_port,
                              config.root.serial_port.baudrate)
            self.serial_port = port
            return True

        self.logger.info("{} Append Treerunner config to "
                         "PORT_CFG".format(log_tag))
        PORT_CFG.update({'PORT_ID_TREERUNNER': {
                'regex': re.compile(r'NXP S32V234'),
                'baud': 115200
            }})

        if port_id not in PORT_CFG.keys():
            self.logger.error(
                "{} Serial port id {} is not valid. Please chose "
                "from {}".format(log_tag, port_id, PORT_CFG.keys()))
            return False

        self.logger.info(
            "{} Trying to auto detect the Treerunner serial port with "
            "id {}".format(log_tag, port_id))

        if "Linux" in platform.system():
            port_name = get_port_name(port_id, ss_pid='/dev/ttyS0')
        else:
            port_name = get_port_name(port_id)
        port_baud = get_port_baud_rate(port_id)

        # Little hack - Treerunner takes a bit of time to boot up
        time.sleep(30)

        if port_name and port_baud:
            self.logger.debug(
                "{} Trerunner serial port {} found "
                "at {}".format(log_tag, port_id, port_name))
            self.serial_port = SerialComm(port_name, port_baud)
            return True

        self.logger.error("{} Failed to auto-detect Treerunner serial "
                          "port".format(log_tag))
        return False

    def exec_command_ssh(self, command, add_profile_conf=True,
                         background=False, timeout=None):
        """
        Executes Command on target through ssh, wait for exit status and
        returns exit_status, stdout and stderr

        :param command: Command to execute
        :param add_profile_conf: Source the shell profile configuration before
            executing the command
        :param background: Executes the command in the background in a
            subshell. The shell does not wait for the command to finish,
            and the return status is 0.
        :param timeout: optional, defaults to None. int value to use as
            the command's channel timeout, in seconds.

        :return: (int, paramiko.ChannelFile, paramiko.ChannelFile)
             capturing (stdout's exit status code, stdout, stderr)
        """
        log_tag = self.get_log_tag()
        if add_profile_conf:
            command = "&&".join([". /etc/profile", command])
        if background:
            command += " &"

        self.logger.info("{} the command to be executed is: {}".format(
            log_tag, command))

        with self.ssh_client as connection:
            stdout, stderr = connection.exec_command(command,
                                                     timeout=timeout)[1:]
            exit_status = stdout.channel.recv_exit_status()
            return exit_status, stdout, stderr

    def clean_up(self):
        """
        General purpose clean-up function
        """
        # TODO: Implement if needed
        pass

    def is_process_running(self, name):
        """
        Checks if at least one instance of a process is running

        :param name: Process name to check for. Must match exactly.
        :return: True or False, depending on if a match is found.
        """
        log_tag = self.get_log_tag()
        self.logger.info("{} Checking to see if the process {} is "
                         "running".format(log_tag, name))
        return self.get_pids(name) is not None

    def reboot(self, *args, **kwargs):
        """
        Reboots the Treerunner board, and starts the sshd server daemon
        """
        log_tag = self.get_log_tag()
        self.logger.info("{} Attempting to reset the Treerunner board"
                         "".format(log_tag))
        cmd = "shutdown > /dev/null 2>&1"
        self.exec_command_ssh(cmd, background=True)
        self.logger.info("{} Waiting for the Treerunner board to come"
                         " back online".format(log_tag))
        time.sleep(30)
        # Start the sshd server daemon
        self.start_sshd_server()

    def collect_core_dump(self, log_dir):
        """No support for Treerunner"""
        pass

    def collect_logs(self, log_dir, label=None, min_t=100, max_t=200):
        """No support for Treerunner"""
        pass

    def collect_slog2info(self, log_dir):
        """
        Collects the slog2info log and then copies it from the
        target to the specified log directory

        :param log_dir: Directory where the log should be copied to
        """
        log_type = "slog2info"
        log_name = "treerunner_slog2info.txt"
        cmd = "slog2info > /tmp/{}".format(log_name)

        self._collect_log(log_type, log_dir, log_name, cmd)

    def collect_pidin(self, log_dir):
        """
        Collects the pidin log and then copies it from the
        target to the specified log directory

        :param log_dir: Directory where the log should be copied to
        """
        log_type = "pidin"
        log_name = "pidin.txt"
        cmd = "pidin > /tmp/{}".format(log_name)

        self._collect_log(log_type, log_dir, log_name, cmd)

    def collect_pidin_ar(self, log_dir):
        """
        Collects the pidin ar log and then copies it from the
        target to the specified log directory

        :param log_dir: Directory where the log should be copied to
        """
        log_type = "pidin ar"
        log_name = "pidin_ar.txt"
        cmd = "pidin ar > /tmp/{}".format(log_name)

        self._collect_log(log_type, log_dir, log_name, cmd)

    def collect_pidin_syspage(self, log_dir, syspage=None):
        """
        Collects the pidin syspage log and then copies it from the
        target to the specified log directory

        :param log_dir: Directory where the log should be copied to
        :param syspage: section to retrieve
        """
        log_type = "pidin syspage"
        log_name = "pidin_syspage.txt"
        if syspage:
            cmd = "pidin syspage={} > /tmp/{}".format(
                syspage, log_name)
        else:
            cmd = "pidin syspage > /tmp/{}".format(log_name)

        self._collect_log(log_type, log_dir, log_name, cmd)

    def collect_top(self, log_dir, iterations=1, timeout=30):
        """
        Collects the top info and then copies it from the
        target to the specified log directory

        :param log_dir: Directory where the log should be copied to
        :param iterations: number of iterations to collect
        :param timeout: timeout to wait for top info collection
        """
        log_type = "top"
        log_name = "top.txt"
        cmd = "top -bd -i {}  > /tmp/{}".format(
            str(iterations), log_name)

        self._collect_log(log_type, log_dir, log_name, cmd,
                          timeout=timeout)

    def collect_hogs(self, log_dir, iterations=1, timeout=30):
        """
        Collects the hogs info and then copies it from the
        target to the specified log directory

        :param log_dir: Directory where the log should be copied to
        :param iterations: number of iterations to collect
        :param timeout: timeout to wait for hogs info collection
        """
        log_type = "hogs"
        log_name = "hogs.txt"
        cmd = "hogs -i {}  > /tmp/{}".format(
            str(iterations), log_name)

        self._collect_log(log_type, log_dir, log_name, cmd,
                          timeout=timeout)

    def collect_nicinfo(self, log_dir):
        """
        Collects the nicinfo and then copies it from the
        target to the specified log directory

        :param log_dir: Directory where the log should be copied to
        """
        log_type = "nicinfo"
        log_name = "nicinfo.txt"
        cmd = "nicinfo  > /tmp/{}".format(log_name)

        self._collect_log(log_type, log_dir, log_name, cmd)

    def collect_tcpdump(self, log_dir, count=10, timeout=30):
        """
        Collects the nicinfo and then copies it from the
        target to the specified log directory

        :param log_dir: Directory where the log should be copied to
        :param count: number of packets to count to
        :param timeout: time to wait. The more packets are counter,
                        the longer the timeout
        """
        log_type = "tcpdump"
        log_name = "tcpdump.txt"
        cmd = "tcpdump -c {} > /tmp/{}".format(count, log_name)

        self._collect_log(log_type, log_dir, log_name, cmd,
                          timeout=timeout, background=False)

    def collect_uname(self, log_dir):
        """
        Collects the uname info and then copies it from the
        target to the specified log directory

        :param log_dir: Directory where the log should be copied to
        """
        log_type = "uname"
        log_name = "uname.txt"
        cmd = "uname -a > /tmp/{}".format(log_name)

        self._collect_log(log_type, log_dir, log_name, cmd)

    def _collect_log(self, log_type, log_dir, log_name, cmd,
                     timeout=10, background=True):
        """
        Helper method to collect specified log

        :param log_type: log to collect
        :param log_dir: log directory on PC
        :param log_name: name for the log
        :param cmd: command to execute
        """
        log_tag = self.get_log_tag()
        target_log = posixpath.join('/tmp/', log_name)
        self.logger.info("{} Attempting to collect a {} log".format(
            log_tag, log_type))
        status, _, _ = self.exec_command_ssh(cmd, background=background)

        if status != 0:
            raise Exception("{} '{}' command did not generate a log "
                            "on the target".format(log_tag, cmd))

        # Wait for the system to finish writing the log
        time.sleep(timeout)
        self.logger.info("{} Attempting to copy generated log from "
                         "the target to the PC".format(log_tag))
        dest = os.path.join(log_dir, log_name)
        with self.ssh_client as ssh_client:
            with ssh_client.open_sftp() as sftp:
                sftp.get(target_log, dest)

        self.logger.info("{} Attempting to delete log from "
                         "target".format(log_tag))
        status, _, _ = self.exec_command_ssh(
            "rm /tmp/{}".format(log_name))

        if status != 0:
            self.logger.error("{} Failed to delete log from "
                              "target".format(log_tag))

        self.logger.info("{} Log collection complete!".format(log_tag))

    def collect_all_logs(self, log_dir):
        """
        Collect all the logs supported by the Treerunner class

        :param log_dir: Directory where the log should be copied to
        """
        log_tag = self.get_log_tag()
        self.logger.info("{} Gathering the list of log collections "
                         "supported".format(log_tag))

        collect_methods = [method_name for method_name in dir(self)
                           if callable(getattr(self, method_name))
                           and "collect_" in method_name and
                           "_collect" not in method_name and
                           "collect_all_logs" not in method_name]

        for method in collect_methods:
            self.logger.info("{} About to {}".format(
                log_tag, method))
            collect_method = getattr(
                self, method, lambda: "Invalid log collection method")
            collect_method(log_dir=log_dir)
