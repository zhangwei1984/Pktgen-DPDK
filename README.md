Pktgen version 2.4.0 using DPDK-1.5.2
=====================================

**Pktgen is a traffic generator powered by Intel's DPDK at 10Gbit wire rate traffic with 64 byte frames.**

**Sounds like 'Packet-Gen'**

**=== Modifications ===**
 - 2.4.1   - Fixed a bug in range packets when 'inc' value is zero use start values.
 - 2.4.0   - Add support for TX tap packets. Change 'tap' command to rxtap and txtap.
 - 2.3.4   - Minor update to help eliminate RX errors and be able to receive at wire rate.
 - 2.3.3   - Update to minor release 1.5.2
 - 2.3.2   - Fixed VLAN detection problem in ARP and special GARP support.
 - 2.3.1   - Getting closer to line rate tx speed.
 - 2.3.0   - Now supports the VLAN encapsulated packets for ARP replies
             Also added a special GARP processing to update the destination MAC
             address to help support a special request for failover support.
             Added support for DPDK 1.5.1
 - 2.2.7   - Updated the code to handle multiple TX queues per port.
 - 2.2.6   - Fixed a crash if the port is not up with link status
 - 2.2.5   - Remove the flow control code as some systems it does not work.
 - 2.2.4   - Fix the inet_h64tom and inet_mtoh64 functions to account for endianness
 - 2.2.3   - range packet fixes for packet size and source/destination mac
 - 2.2.2   - Minor performance changes for receive packet performance.
 - 2.2.1   - Change MAC address from XXXX:XXXX:XXXX to XX:XX:XX:XX:XX:XX format
             Fixed Pktgen to allow packet changes without having to restart the tool.
 - 2.2.0   - Update to DPDK 1.5.0
**=====================**

Please look at the product eval PDF and the 3rd party PDF for license information.

---
**Copyright &copy; \<2010-2014\>, Intel Corporation All rights reserved.**
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

 - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in
   the documentation and/or other materials provided with the
   distribution.

 - Neither the name of Intel Corporation nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 OF THE POSSIBILITY OF SUCH DAMAGE.

---
**Copyright &copy; \<2010-2014\>, Wind River Systems, Inc.**

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1) Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

 2) Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation and/or
    other materials provided with the distribution.

 3) Neither the name of Wind River Systems nor the names of its contributors may be
    used to endorse or promote products derived from this software without specific
    prior written permission.

 4) The screens displayed by the application must contain the copyright notice as defined
    above and can not be removed without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 Pktgen: Created 2010 by Keith Wiles @ windriver.com
 ---

**====================== Part of the README file ===========================**

Pktgen
------
Copyright &copy; \<2010-2014\>, Wind River Systems, Inc.

       Contact me if you have questions @ pktgen.dpdk@gmail.com<br>
       Source can be found on GitHub: http://github.com/Pktgen/Pktgen-DPDK

README for setting up Pktgen with DPDK on Ubuntu 10.04 to 12.10 desktop, it
should work on most Linux systems as long as the kernel has hugeTLB support.

```
Note: Tested with Ubuntu 12.10 kernel version
Linux 3.5.0-25-generic #39-Ubuntu SMP Mon Feb 25 18:26:58 UTC 2013 x86_64
```

I am using Ubuntu 12.10 x86_64 (64 bit support) for running pktgen/DPDK on a
Westmere Dual socket board running at 2.4GHz with 12GB of ram 6GB per socket.
The current kernel version is 3.5.0-25 (as of 2013-03-18) support, but should
work on just about any new Linux kernel version.

To get hugeTLB support your Linux kernel must be at least 2.6.33 and in the
DPDK documents it talks about how you can upgrade your Linux kernel.

The pktgen output display needs 132 columns and about 42 lines to display
correctly. I am using an xterm of 132x42, but you can have a larger display
and maybe a bit smaller. If you are displaying more then 4-6 ports then you
will need a wider display. The pktgen allows you to view a set ports if they
do not all fit on the screen at one time via the 'page' command in pktgen.
Type 'help' at the 'pktgen>' prompt to see the complete pktgen commands.
Pktgen uses VT100 control codes or escape codes to display the screens,
which means your terminal must support VT100. The Hyperterminal in windows is not
going to work for Pktgen as it has a few problems with VT100 codes.

The pktgen program as built can send up to 16 packets per port in a sequence
and you can configure a port using the 'seq' pktgen command. A script file
can be loaded from the shell command line via the -f option and you can 'load'
a script file from within pktgen as well.

Packet Buffer types and usages:
-------------------------------

Pktgen has four different sets of packet buffers to be used for a given type of
data to be sent on the wire per port. The four buffer types are single packet, 
16 sequence packets, range packets and PCAP packet buffers.

The single packet buffer is defined by the information on the main screen. Each port
in the system has a column of information on the screen to each single packet. The
information toward the bottom of the column is the IP address, port numbers and MAC
addresses for the single packet. When you do a 'start N' command and you have not
setup any other type of packets to send on that port it will send the packet defined
on the screen.

The next type of buffers are the sequence packets, which is a set of packet (up to 16)
for each port. Each port can have a different set of sequence packets or be able to
send other types of packets like single packet type. If you execute the command 'page seq'
it will display the port 0 sequence packets and using the command 'port X' you can change
to another port while in the sequence screen. Also using the set <portlist> seqCnt <value>
command you can assign 1 to 16 different packets to the port. Using the commands you can
set a given sequence packet information for a given port. The command starts with 'seq'
and you can look at the help screen to determine the command usage.

The next type of packet buffers is the range packets. The user is allowed to set up
ranges for the packets by adjusting the values or ranges for a number of fields in the
packet like IP address, MAC address, port numbers, packet sizes and a number of other
options. Using the range command 'page range' you can look at the port values the range
packets for a given port and using 'port X' to switch to different ports. This option
needs to be enabled per port just like sequence packets must be enabled. The flags
value at the top of the screen on each port help the user to see how they are configured.

The last type of packet buffers is the PCAP packets. Pktgen will load a PCAP file from
the command line for a given port using X:filename.pcap where X is the port number. To
have Pktgen send the PCAP file you must also enable the port to send that PCAP file. You
can switch from one type of packet buffer to another at will and Pktgen will remember
the configuration of the other packet buffers in the system. If you want to save the
configuration you need to a 'save' command to write the configuration to a file. You can
then load that file again using the load command or place it on the command line.

Example screen of Pktgen:
-------------------------

```
-  Ports 0-3 of 4 ** Main Page **   Copyright (c) <2010-2014>, Wind River Systems, Inc., Powered by Intel® DPDK
  Flags:Port     :    P-----------:0    P-----------:1    P-----------:2    P-----------:3
Link State       :     <UP-10000-FD>     <UP-10000-FD>     <UP-10000-FD>     <UP-10000-FD>   ---TotalRate---
Pkts/s  Rx       :                 0                 0                 0                 0                 0
        Tx       :                 0                 0                 0                 0                 0
MBits/s Rx/Tx    :               0/0               0/0               0/0               0/0               0/0
Broadcast        :                 0                 0                 0                 0
Multicast        :                 0                 0                 0                 0
  64 Bytes       :                 0                 0                 0                 0
  65-127         :                 0                 0                 0                 0
  128-255        :                 0                 0                 0                 0
  256-511        :                 0                 0                 0                 0
  512-1023       :                 0                 0                 0                 0
  1024-1518      :                 0                 0                 0                 0
  Runts          :                 0                 0                 0                 0
  Jumbo          :                 0                 0                 0                 0
Errors Rx/Tx     :               0/0               0/0               0/0               0/0
Total Rx Pkts    :                 0                 0                 0                 0
      Tx Pkts    :                 0                 0                 0                 0
      Rx MBs     :                 0                 0                 0                 0
      Tx MBs     :                 0                 0                 0                 0
ARP/ICMP Pkts    :               0/0               0/0               0/0               0/0
No Mbufs/unknown :               0/0               0/0               0/0               0/0
                 :
Tx Count/% Rate  :      Forever/100%      Forever/100%      Forever/100%      Forever/100%
PktSize/Tx Burst :            64/128            64/128            64/128            64/128
Src/Dest Port    :         1234/5678         1234/5678         1234/5678         1234/5678
Pkt Type:VLAN ID :     IPv4/TCP:0001     IPv4/TCP:0001     IPv4/TCP:0001     IPv4/TCP:0001
Dst  IP Address  :       192.168.1.1       192.168.0.1       192.168.3.1       192.168.2.1
Src  IP Address  :    192.168.0.1/24    192.168.1.1/24    192.168.2.1/24    192.168.3.1/24
Dst MAC Address  :    001b:218e:b1e9    001b:218e:b1e8    001b:218e:b761    001b:218e:b760
Src MAC Address  :    001b:218e:b1e8    001b:218e:b1e9    001b:218e:b760    001b:218e:b761
- Pktgen Ver:2.3.1(DPDK-1.5.2) --------------------------------------------------------------------------------------







pktgen> quit
```

---
```
Here is the help screen:

   *** Help Information for Pktgen ***         Copyright (c) <2010-2014>, Wind River Systems, Inc.
set <portlist> <xxx> value         - Set a few port values
  <portlist>                       - a list of ports as 2,4,6-9,12 or the word 'all'
  <xxx>          count             - number of packets to transmit
                 size              - size of the packet to transmit
                 rate              - Packet rate in percentage
                 burst             - number of packets in a burst
                 sport             - Source port number for TCP
                 dport             - Destination port number for TCP
                 prime             - Set the number of packets to send on prime command
                 seqCnt            - Set the number of packet in the sequence to send
seq <seq#> <portlist> dst-Mac src-Mac dst-IP src-IP sport dport ipv4|ipv6 udp|tcp|icmp vid pktsize
                                   - Set the sequence packet information, make sure the src-IP
                                     has the netmask value eg 1.2.3.4/24
save filename                      - Save a configuration file using the filename
ppp [1-6]                          - Set the number of ports displayed per page
icmp.echo <portlist> <state>       - Enable/disable ICMP echo responses per port
send arp req|grat <portlist>       - Send a ARP request or gratuitous ARP on a set of ports
set mac <portlist> etheraddr       - Set MAC addresses 00:11:22:33:44:55
                                     You can use 0011:2233:4455 format as well
mac_from_arp <state>               - Set the option to get MAC from ARP request
set udp|tcp|icmp <portlist>        - Set the packet type UDP or TCP or ICMP per port
set ipv4|ipv6 <portlist>           - Set the packet type to IPv4 or IPv6
set ip src|dst <portlist> ipaddr   - Set IP addresses
geometry <geom>                    - Set the display geometry Columns by Rows (ColxRow)
tap <portlist> <state>             - Enable/disable tap interface support
vlan <portlist> <state>            - Enable/disable sending VLAN ID in packets
vlanid <portlist> <vlanid>         - Set the VLAN ID for the portlist
pcap <portlist> <state>            - Enable or Disable sending pcap packets on a portlist
pcap.show                          - Show the PCAP information
pcap.index                         - Move the PCAP file index to the given packet number,  0 - rewind, -1 - end of file
script <filename>                  - Execute the Lua script code in file (www.lua.org).
ping4 <portlist>                   - Send a IPv4 ICMP echo request on the given portlist

page [0-7]|range|config|seq|pcap|next|cpu- Show the port pages or configuration or sequence page
     [0-7]                         - Page of different ports
     range                         - Display the range packet page
     config                        - Display the configuration page (not used)
     pcap                          - Display the pcap page
     cpu                           - Display some information about the CPU system
     next                          - Display next page of PCAP packets.
     sequence | seq                - sequence will display a set of packets for a given port
                                     Note: use the 'port <number>' to display a new port sequence
port <number>                      - Sets the sequence of packets to display for a given port
process <portlist> <state>         - Enable or Disable processing of ARP/ICMP/IPv4/IPv6 packets
garp <portlist> <state>            - Enable or Disable processing of GARP packets
blink <portlist> <state>           - Blink the link led on the given port list
start <portlist>                   - Start transmitting packets
stop <portlist>                    - Stop transmitting packets
stp                                - Stop all ports from transmitting
str                                - Start all ports transmitting

screen stop|start                  - stop/start updating the screen and unlock/lock window
off                                - screen off shortcut
on                                 - screen on shortcut
prime <portlist>                   - Transmit N packets on each port listed. See set prime command above
delay milliseconds                 - Wait a number of milliseconds for scripting commands
sleep seconds                      - Wait a number of seconds for scripting commands
load <path>                        - Load a command/script file from the given path
pci show                           - Show the PCI bus devices
clear <portlist>                   - Clear the statistics
clr                                - Clear all Statistices
cls                                - Clear the screen
reset <portlist>                   - Reset the configuration to the default
rst                                - Reset the configuration for all ports
help                               - Display this help message
quit                               - Quit the Pktgen program

  -- Setup the packet range values --
dst.mac start <portlist> etheraddr - Set destination MAC address start
src.mac start <portlist> etheraddr - Set source MAC address start
src.ip start <portlist> ipaddr     - Set source IP start address
src.ip min <portlist> ipaddr       - Set source IP minimum address
src.ip max <portlist> ipaddr       - Set source IP maximum address
src.ip inc <portlist> ipaddr       - Set source IP increment address
dst.ip start <portlist> ipaddr     - Set destination IP start address
dst.ip min <portlist> ipaddr       - Set destination IP minimum address
dst.ip max <portlist> ipaddr       - Set destination IP maximum address
dst.ip inc <portlist> ipaddr       - Set destination IP increment address
src.port start <portlist> value    - Set source port start address
src.port min <portlist> value      - Set source port minimum address
src.port max <portlist> value      - Set source port maximum address
src.port inc <portlist> value      - Set source port increment address
dst.port start <portlist> value    - Set source port start address
dst.port min <portlist> value      - Set source port minimum address
dst.port max <portlist> value      - Set source port maximum address
dst.port inc <portlist> value      - Set source port increment address
vlan.id start <portlist> value     - Set vlan id start address
vlan.id min <portlist> value       - Set vlan id minimum address
vlan.id max <portlist> value       - Set vlan id maximum address
vlan.id inc <portlist> value       - Set vlan id increment address
pkt.size start <portlist> value    - Set vlan id start address
pkt.size min <portlist> value      - Set vlan id minimum address
pkt.size max <portlist> value      - Set vlan id maximum address
pkt.size inc <portlist> value      - Set vlan id increment address
range <portlist> <state>           - Enable or Disable the given portlist for sending a range of packets

Notes: <state> - Use enable|disable or on|off to set the state.
       Flags: P------------ - Promiscuous mode enabled
               E            - ICMP Echo enabled
                A           - Send ARP Request flag
                 G          - Send Gratuitous ARP flag
                  C         - TX Cleanup flag
                   p        - PCAP enabled flag
                    S       - Send Sequence packets enabled
                     R      - Send Range packets enabled
                      D     - DPI Scanning enabled (If Enabled)
                       I    - Process packets on input enabled
                        T   - Using TAP interface for this port
                         V  - Send VLAN ID tag
                          g - Special GARP processing of packets
   <Press Return to Continue>
```
---

Thanks
++Keith
