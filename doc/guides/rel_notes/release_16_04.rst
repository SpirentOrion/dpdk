DPDK Release 16.04
==================


**Read this first**

The text below explains how to update the release notes.

Use proper spelling, capitalization and punctuation in all sections.

Variable and config names should be quoted as fixed width text: ``LIKE_THIS``.

Build the docs and view the output file to ensure the changes are correct::

   make doc-guides-html

   firefox build/doc/html/guides/rel_notes/release_16_04.html


New Features
------------

This section should contain new features added in this release. Sample format:

* **Add a title in the past tense with a full stop.**

  Add a short 1-2 sentence description in the past tense. The description
  should be enough to allow someone scanning the release notes to understand
  the new feature.

  If the feature adds a lot of sub-features you can use a bullet list like this.

  * Added feature foo to do something.
  * Enhanced feature bar to do something else.

  Refer to the previous release notes for examples.

* **Added function to check primary process state.**

  A new function ``rte_eal_primary_proc_alive()`` has been added
  to allow the user to detect if a primary process is running.
  Use cases for this feature include fault detection, and monitoring
  using secondary processes.

* **Enabled bulk allocation of mbufs.**

  A new function ``rte_pktmbuf_alloc_bulk()`` has been added to allow the user
  to allocate a bulk of mbufs.

* **Restored vmxnet3 Tx data ring.**

  Tx data ring has been shown to improve small pkt forwarding performance
  on vSphere environment.

* **Added vmxnet3 Tx L4 checksum offload.**

  Support TCP/UDP checksum offload.

* **Added vmxnet3 TSO support.**

* **Added vmxnet3 support for jumbo frames.**

  Added support for linking multi-segment buffers together to
  handle Jumbo packets.

* **Virtio 1.0.**

  Enabled virtio 1.0 support for virtio pmd driver.

* **Supported virtio for ARM.**

  Enabled virtio support for armv7/v8. Tested for arm64.
  Virtio for arm support VFIO-noiommu mode only.
  Virtio can work with other non-x86 arch too like powerpc.

* **Supported virtio offload in vhost-user.**

  Add the offload and negotiation of checksum and TSO between vhost-user and
  vanilla Linux virtio guest.

* **Added vhost-user live migration support.**

* **Added multicast promiscuous mode support on VF for ixgbe.**

  Added multicast promiscuous mode support on ixgbe VF driver. So all the VFs
  can receive the multicast packets.

  Please note if we want to use this promiscuous mode, we need both PF and VF
  driver to support it. The reason is this VF feature is configured on PF.
  If use kernel PF driver + dpdk VF driver, make sure kernel PF driver support
  VF multicast promiscuous mode. If use dpdk PF + dpdk VF, better make sure PF
  driver is the same version as VF.

* **Added support for E-tag on X550.**

  E-tag is defined in 802.1br. Please reference
  http://www.ieee802.org/1/pages/802.1br.html.

  This feature is for VF, but the settings are on PF. It means
  the CLIs should be used on PF, but some of their effects will be shown on VF.
  The forwarding of E-tag packets based on GRP and E-CID_base will have effect
  on PF. Theoretically the E-tag packets can be forwarded to any pool/queue.
  But normally we'd like to forward the packets to the pools/queues belonging
  to the VFs. And E-tag insertion and stripping will have effect on VFs. When
  VF receives E-tag packets, it should strip the E-tag. When VF transmits
  packets, it should insert the E-tag. Both can be offloaded.

  When we want to use this E-tag support feature, the forwarding should be
  enabled to forward the packets received by PF to indicated VFs. And insertion
  and stripping should be enabled for VFs to offload the effort to HW.

  * Support E-tag offloading of insertion and stripping.
  * Support Forwarding E-tag packets to pools based on
    GRP and E-CID_base.

* **Added support for VxLAN & NVGRE checksum off-load on X550.**

  * Added support for VxLAN & NVGRE RX/TX checksum off-load on
    X550. RX/TX checksum off-load is provided on both inner and
    outer IP header and TCP header.
  * Added functions to support VxLAN port configuration. The
    default VxLAN port number is 4789 but this can be updated
    programmatically.

* **Added new X550EM_a devices.**

  Added new X550EM_a devices and their mac types, X550EM_a and X550EM_a_vf.
  Updated the code to use the new devices and mac types.

* **Added x550em_x V2 device support.**

  Only x550em_x V1 was supported before. Now V2 is supported.
  A mask for V1 and V2 is defined and used to support both.

* **Supported link speed auto-negotiation on X550EM_X**

  Normally the auto-negotiation is supported by FW. SW need not care about
  that. But on x550em_x, FW doesn't support auto-neg. As the ports of x550em_x
  are 10G, if we connect the port with a peer which is 1G, the link will always
  be down.
  We added the support of auto-neg by SW to avoid this link down issue.

* **Added sw-firmware sync on X550EM_a.**

  Added support for sw-firmware sync for resource sharing.
  Use the PHY token, shared between sw-fw for PHY access on X550EM_a.

* **Updated the i40e base driver.**

  The i40e base driver was updated with changes including the
  following:

  * Use Rx control AQ commands to read/write Rx control registers.
  * Add new X722 device IDs, and removed X710 one was never used.
  * Expose registers for HASH/FD input set configuring.

* **Enabled PCI extended tag for i40e.**

  It enabled extended tag by checking and writing corresponding PCI config
  space bytes, to boost the performance. In the meanwhile, it deprecated the
  legacy way via reading/writing sysfile supported by kernel module igb_uio.

* **Added i40e support for setting mac addresses.**

* **Added dump of i40e registers and EEPROM.**

* **Supported ether type setting of single and double VLAN for i40e**

* **Added VMDQ DCB mode in i40e.**

  Added support for DCB in VMDQ mode to i40e driver.

* **Added i40e VEB switching support.**

* **Added fm10k Rx interrupt support.**

* **Optimized fm10k Tx.**

  * Free multiple mbufs at a time to reduce freeing mbuf cycles.

* **Handled error flags in fm10k vector Rx.**

  Parse err flags in Rx desc and set error bits in mbuf with vector instructions.

* **Added fm10k FTAG based forwarding support.**

* **Added mlx5 flow director support.**

  Added flow director support (RTE_FDIR_MODE_PERFECT and
  RTE_FDIR_MODE_PERFECT_MAC_VLAN).

  Only available with Mellanox OFED >= 3.2.

* **Added mlx5 RX VLAN stripping support.**

  Added support for RX VLAN stripping.

  Only available with Mellanox OFED >= 3.2.

* **Changed szedata2 type of driver from vdev to pdev.**

  Previously szedata2 device had to be added by ``--vdev`` option.
  Now szedata2 PMD recognises the device automatically during EAL
  initialization.

* **Added szedata2 functions for setting link up/down.**

* **Added szedata2 promiscuous and allmulticast modes.**

* **Added af_packet dynamic removal function.**

  Af_packet device can now be detached using API, like other PMD devices.

* **Increased number of next hops for LPM IPv4 to 2^24.**

  The next_hop field is extended from 8 bits to 24 bits for IPv4.

* **Added support of SNOW 3G (UEA2 and UIA2) for Intel Quick Assist devices.**

  Enabled support for SNOW 3G wireless algorithm for Intel Quick Assist devices.
  Support for cipher only, hash only is also provided
  along with alg-chaining operations.

* **Added SNOW3G SW PMD.**

  A new Crypto PMD has been added, which provides SNOW 3G UEA2 ciphering
  and SNOW3G UIA2 hashing.

* **Added AES GCM PMD.**

  Added new Crypto PMD to support AES-GCM authenticated encryption and
  authenticated decryption in SW.

* **Added NULL Crypto PMD**

  Added new Crypto PMD to support null crypto operations in SW.

* **Improved IP Pipeline Application.**

  The following features have been added to ip_pipeline application;

  * Added CPU utilization measurement and idle cycle rate computation.
  * Added link idenfication support through existing port-mask option or by
    specifying PCI device in every LINK section in the configuration file.
  * Added load balancing support in passthrough pipeline.

* **Added IPsec security gateway example.**

  New application implementing an IPsec Security Gateway.


Resolved Issues
---------------

This section should contain bug fixes added to the relevant sections. Sample format:

* **code/section: Fixed issue in the past tense with a full stop.**

  Add a short 1-2 sentence description of the resolved issue in the past tense.
  The title should contain the code/lib section like a commit message.
  Add the entries in alphabetic order in the relevant sections below.


EAL
~~~


Drivers
~~~~~~~

* **ethdev: Fixed byte order consistency between fdir flow and mask.**

  Fixed issue in ethdev library that the structure for setting
  fdir's mask and flow entry was not consistent in byte ordering.

* **cxgbe: Fixed crash due to incorrect size allocated for RSS table.**

  Fixed a segfault that occurs when accessing part of port 0's RSS
  table that gets overwritten by subsequent port 1's part of the RSS
  table due to incorrect size allocated for each entry in the table.

* **cxgbe: Fixed setting wrong device MTU.**

  Fixed an incorrect device MTU being set due to ethernet header and
  CRC lengths being added twice.

* **ixgbe: Fixed zeroed VF mac address.**

  Resolved an issue where VF mac address is zeroed out in cases where the VF
  driver is loaded while the PF interface is down.
  The solution is to only set it when we get an ACK from the PF.

* **ixgbe: Fixed setting flow director flag twice.**

  Resolved an issue where packets were being dropped when switching to perfect
  filters mode.

* **ixgbe: Set MDIO speed after MAC reset.**

  The MDIO clock speed must be reconfigured after the MAC reset. The MDIO clock
  speed becomes invalid, therefore the driver reads invalid PHY register values.
  The driver now set the MDIO clock speed prior to initializing PHY ops and
  again after the MAC reset.

* **i40e: Generated MAC address for each VFs.**

  It generates a MAC address for each VFs during PF host initialization,
  and keeps the VF MAC address the same among different VF launch.

* **i40e: Fixed failure of reading/writing Rx control registers.**

  Fixed i40e issue of failing to read/write rx control registers when
  under stress with traffic, which might result in application launch
  failure.

* **mlx5: Fixed possible crash during initialization.**

  A crash could occur when failing to allocate private device context.

* **mlx5: Added port type check.**

  Done to prevent port initialization on non-Ethernet link layers and
  to report an error.

* **mlx5: Applied VLAN filtering to broadcast and IPv6 multicast flows.**

  Prevented reception of multicast frames outside of configured VLANs.

* **mlx5: Fixed RX checksum offload in non L3/L4 packets.**

  Fixed report of bad checksum for packets of unknown type.

* **aesni_mb: Fixed wrong return value when creating a device.**

  cryptodev_aesni_mb_init() was returning the device id of the device created,
  instead of 0 (when success), that rte_eal_vdev_init() expects.
  This made impossible the creation of more than one aesni_mb device
  from command line.

* **qat: Fixed AES GCM decryption.**

  Allowed AES GCM on the cryptodev API, but in some cases gave invalid results
  due to incorrect IV setting.


Libraries
~~~~~~~~~

* **hash: Fixed CRC32c hash computation for non multiple of 4 bytes sizes.**

  Fix crc32c hash functions to return a valid crc32c value for data lengths
  not multiple of 4 bytes.

* **librte_port: Fixed segmentation fault for ring and ethdev writer nodrop.**

  Fixed core dump issue on txq and swq when dropless is set to yes.


Examples
~~~~~~~~

* **l3fwd-power: Fixed memory leak for non-IP packet.**

  Fixed issue in l3fwd-power where, on receiving packets of types
  other than IPv4 or IPv6, the mbuf was not released, and caused
  a memory leak.

* **examples/vhost: Fixed frequent mbuf allocation failure.**

  vhost-switch often fails to allocate mbuf when dequeue from vring because it
  wrongly calculates the number of mbufs needed.


Other
~~~~~


Known Issues
------------

This section should contain new known issues in this release. Sample format:

* **Add title in present tense with full stop.**

  Add a short 1-2 sentence description of the known issue in the present
  tense. Add information on any known workarounds.


API Changes
-----------

This section should contain API changes. Sample format:

* Add a short 1-2 sentence description of the API change. Use fixed width
  quotes for ``rte_function_names`` or ``rte_struct_names``. Use the past tense.

* The functions ``rte_eth_dev_udp_tunnel_add`` and ``rte_eth_dev_udp_tunnel_delete``
  have been renamed into ``rte_eth_dev_udp_tunnel_port_add`` and
  ``rte_eth_dev_udp_tunnel_port_delete``.

* The ``outer_mac`` and ``inner_mac`` fields in structure
  ``rte_eth_tunnel_filter_conf`` are changed from pointer to struct in order
  to keep code's readability.

* The fields in ethdev structure ``rte_eth_fdir_masks`` were changed
  to be in big endian.

* A parameter ``vlan_type`` has been added to the function
  ``rte_eth_dev_set_vlan_ether_type``.

* AF_packet device init function is no longer public. Device should be attached
  with API.

* The LPM ``next_hop`` field is extended from 8 bits to 24 bits for IPv4
  while keeping ABI compatibility.

* A new ``rte_lpm_config`` structure is used so LPM library will allocate
  exactly the amount of memory which is necessary to hold application’s rules.
  The previous ABI is kept for compatibility.

* The prototype for the pipeline input port, output port and table action
  handlers are updated: the pipeline parameter is added,
  the packets mask parameter has been either removed or made input-only.


ABI Changes
-----------

* Add a short 1-2 sentence description of the ABI change that was announced in
  the previous releases and made in this release. Use fixed width quotes for
  ``rte_function_names`` or ``rte_struct_names``. Use the past tense.

* The RETA entry size in ``rte_eth_rss_reta_entry64`` has been increased
  from 8-bit to 16-bit.

* The cmdline buffer size has been increase from 256 to 512.


Shared Library Versions
-----------------------

Update any library version updated in this release and prepend with a ``+`` sign.

The libraries prepended with a plus sign were incremented in this version.

.. code-block:: diff

   + libethdev.so.3
     librte_acl.so.2
     librte_cfgfile.so.2
   + librte_cmdline.so.2
     librte_distributor.so.1
     librte_eal.so.2
     librte_hash.so.2
     librte_ip_frag.so.1
     librte_ivshmem.so.1
     librte_jobstats.so.1
     librte_kni.so.2
     librte_kvargs.so.1
     librte_lpm.so.2
     librte_mbuf.so.2
     librte_mempool.so.1
     librte_meter.so.1
   + librte_pipeline.so.3
     librte_pmd_bond.so.1
     librte_pmd_ring.so.2
     librte_port.so.2
     librte_power.so.1
     librte_reorder.so.1
     librte_ring.so.1
     librte_sched.so.1
     librte_table.so.2
     librte_timer.so.1
     librte_vhost.so.2
