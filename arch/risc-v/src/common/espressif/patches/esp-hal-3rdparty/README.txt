Patches applied to esp-hal-3rdparty after it is cloned at ESP_HAL_3RDPARTY_VERSION
(see the chip/$(ESP_HAL_3RDPARTY_REPO) rule in Make.defs), in file name order.

0001-bt-vhci-copy-host-acl-outside-assert.patch
  hci_driver_vhci_host_tx() copied each host ACL packet into the controller's
  mbuf inside assert().  NuttX builds without assertions define NDEBUG, so the
  copy was compiled out and the controller received empty ACL packets: no
  GATT/ATT answer from an ESP32-C6 or ESP32-H2 ever went over the air.
