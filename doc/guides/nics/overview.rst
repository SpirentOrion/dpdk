..  BSD LICENSE
    Copyright 2016 6WIND S.A.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.
    * Neither the name of 6WIND S.A. nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
    A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Overview of Networking Drivers
==============================

The networking drivers may be classified in two categories:

- physical for real devices
- virtual for emulated devices

Some physical devices may be shaped through a virtual layer as for
SR-IOV.
The interface seen in the virtual environment is a VF (Virtual Function).

The ethdev layer exposes an API to use the networking functions
of these devices.
The bottom half part of ethdev is implemented by the drivers.
Thus some features may not be implemented.

There are more differences between drivers regarding some internal properties,
portability or even documentation availability.
Most of these differences are summarized below.

.. _table_net_pmd_features:

.. raw:: html

   <style>
      table#id1 th {
         font-size: 80%;
         white-space: pre-wrap;
         text-align: center;
         vertical-align: top;
         padding: 3px;
      }
      table#id1 th:first-child {
         vertical-align: bottom;
      }
      table#id1 td {
         font-size: 70%;
         padding: 1px;
      }
      table#id1 td:first-child {
         padding-left: 1em;
      }
   </style>

.. table:: Features availability in networking drivers

   ==================== = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
   Feature              a b b b c e e i i i i i i i i i i f f m m m n n p r s v v v x
                        f n n o x 1 n 4 4 4 4 g g x x x x m m l l p f u c i z i i m e
                        p x x n g 0 i 0 0 0 0 b b g g g g 1 1 x x i p l a n e r r x n
                        a 2 2 d b 0 c e e e e   v b b b b 0 0 4 5 p   l p g d t t n v
                        c x x i e 0     . v v   f e e e e k k     e         a i i e i
                        k   v n         . f f       . v v   .               t o o t r
                        e   f g         .   .       . f f   .               a   . 3 t
                        t               v   v       v   v   v               2   v
                                        e   e       e   e   e                   e
                                        c   c       c   c   c                   c
   ==================== = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
   link status                  X     X X                                   X
   link status event                  X X
   Rx interrupt                       X X X X
   queue start/stop             X     X X X X                               X
   MTU update                   X
   jumbo frame                  X     X X X X
   scattered Rx                 X     X X X X                               X
   LRO
   TSO                          X     X X X X
   promiscuous mode             X     X X X X                               X
   allmulticast mode            X     X X X X                               X
   unicast MAC filter                 X X X X
   multicast MAC filter               X X X X
   RSS hash                     X     X X X X
   RSS key update                     X X X X
   RSS reta update                    X X X X
   VMDq                               X X
   SR-IOV                             X X
   DCB                                X X
   VLAN filter                        X X X X
   ethertype filter                   X X
   n-tuple filter
   SYN filter
   tunnel filter                      X X
   flexible filter
   hash filter                        X X X X
   flow director                      X X
   flow control                 X     X X
   rate limitation
   traffic mirroring                  X X
   CRC offload                  X     X   X
   VLAN offload                 X     X   X
   QinQ offload                       X   X
   L3 checksum offload          X     X   X
   L4 checksum offload          X     X   X
   inner L3 checksum                  X   X
   inner L4 checksum                  X   X
   packet type parsing          X     X   X
   timesync                           X X
   basic stats                  X     X X X X                               X
   extended stats                     X X X X
   stats per queue              X                                           X
   EEPROM dump
   registers dump
   multiprocess aware                 X X X X
   BSD nic_uio                  X     X X X X
   Linux UIO                    X     X X X X
   Linux VFIO                   X     X X X X
   other kdrv                                                               X
   ARMv7
   ARMv8
   Power8
   TILE-Gx
   x86-32                       X     X X X X
   x86-64                       X     X X X X                               X
   usage doc                    X                                           X
   design doc
   perf doc
   ==================== = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
