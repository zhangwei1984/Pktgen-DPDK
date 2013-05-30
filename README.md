Pktgen-DPDK
===========

Pktgen is a traffic generator powered by Intel's DPDK.<br>
Please look at the product eval PDF and the 3rd party PDF for license information.

---
Copyright (c) <2010-2013>, Intel Corporation All rights reserved.
 
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
Copyright (c) <2010-2013>, Wind River Systems, Inc.

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

====================== Part of the README file ===========================

                          *** Pktgen ****
          Copyright (c) <2010-2013>, Wind River Systems, Inc.

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
-  Ports 0-3 of 4 ** Main Page **   Copyright (c) <2010-2013>, Wind River Systems, Inc., Powered by Intel® DPDK
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
- Pktgen Ver:1.9.1(DPDK-1.3.0) --------------------------------------------------------------------------------------







pktgen> quit
```

Thanks
++Keith
