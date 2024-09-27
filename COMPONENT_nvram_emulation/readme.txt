NVRAM Emulation Library

    This library supports NVRAM emulation in RAM for CYW955513EVK-01 and CYW920835M2EVB-01 kits
    when modeling operation without flash. APIs are provided to allow these platforms to emulate
    NVRAM in RAM. This also enables sample applications to demonstrate functionality normally
    requiring NVRAM access, for example link key storage and retrieval, with minimal source code changes.

    Check if btsdk-driver support is included with the application. If not, add it by using
    the library-manager. Enable COMPONENTS+=nvram_emulation in the makefile to include the
    library source code in the application build. In C source files where the wiced_hal_*_nvram
    API will be used, make changes as follows:

    /* add header */
    #ifdef COMPONENT_nvram_emulation
    #include "nvram_emulation_mem.h"
    #endif

    This will redefine the wiced_hal_*_nvram calls in the existing source code to call nvram_emulation
    functions instead.

    Also add the following call in application start to initialize the library:

    /* initialize the library before using */
    #ifdef COMPONENT_nvram_emulation
        nvram_emulation_mem_init();
    #endif

    The library defines a weak symbol nvram_emulation_mem_cfg, which is an array of nvram_emulation_mem_buf_cfg_t
    structures used to define the NVRAM emulation storage in RAM. Override this symbol with another
    of the same type to optimize the storage needed by the application.

    The library supports older (v1.X) and newer (v3.X) AIROC Bluetooth stack versions that have some API differences.
    The stack is built into the device used in the development kit and is a fixed version for each device.
    The newer stack can allocate buffers from a heap. The older stack only uses buffers from pools that
    are created when the application starts. Kits using the older stack configure the wiced_bt_cfg_settings_t
    structure passed to wiced_bt_stack_init() when the app starts to allow additional buffer pools.
    The wiced_bt_cfg_settings_t member "max_number_of_buffer_pools" is increased to allow additional
    pools configured in nvram_emulation_mem_buf_cfg_t mentioned above. This configuration change is
    not needed for the newer stack.

    This library also provides an example for solutions that will use an external host to store
    and retrieve data from host storage. The LE_Hello_Sensor Code Example demonstrates how to use
    this library to backup data to a Host MCU. To use this feature, define NVRAM_EMULATION_HCI=1 in
    the application makefile by adding "CY_APP_DEFINES+=-DNVRAM_EMULATION_HCI=1". For HCI protocol
    details refer to https://infineon.github.io/btsdk-docs/BT-SDK/AIROC-HCI-Control-Protocol.pdf
    regarding commands HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA and HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA
    and event HCI_CONTROL_EVENT_NVRAM_DATA.

    The application must be configured to support the HCI transport buffers and callbacks. This configuration
    is found in the wiced_transport_cfg_t structure that is passed to wiced_transport_init() when the
    application starts. To enable HCI data transfer, configure the rx_buff_pool_cfg member to specify a
    receive buffer size and count. Further, initialize the p_status_handler, p_data_handler, and optionally
    p_tx_complete_cback members with callback handling functions. These callbacks can support application
    specific HCI transfers as well as calling the corresponding nvram_emulation library handlers:
    nvram_emulation_transport_status_handler (call from p_status_handler) and
    nvram_emulation_transport_rx_data_handler (call from p_data_handler). The function pointer
    p_status_handler is called when the HCI transport is ready after device start. The
    nvram_emulation_transport_status_handler sends an indication to the Host MCU that the device is ready.
    The function pointer p_data_handler is called when HCI packets are received. The function
    nvram_emulation_transport_rx_data_handler processes received packets. It returns HCI_CONTROL_STATUS_SUCCESS
    and frees the buffer if the packet opcode was HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA.

    A simple script "nvram_emulation_backup.py" is included to demonstrate the nvram backup operation via HCI
    with a host device. After setting the default serial port parameters in "wiced_hci_default_config.py",
    invoke the script with "python nvram_emulation_backup.py", for example. The python script reads a file
    "nvram.json" at start up to find recorded nvram data from the previous session. If any nvram items are found
    in the file they are sent via HCI using HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA packets. This restores the
    emulated nvram to the state from the previous session.

    After the restore operation is complete, the script monitors the HCI, looking for HCI_CONTROL_EVENT_NVRAM_DATA
    packets. The nvram_emulation library will send these packets whenever an nvram storage item is modified
    or created by the embedded application. When these packets are received, the script will update any items that
    have a matching vs_id in the nvram.json file or create new items if they don't already exist. This allows
    the script to keep a backup copy of nvram data in the nvram.json file.

    To use the script with an embedded application, follow this procedure:
    1. Start from a clean state: delete nvram.json and forget any pairing keys in the peer Bluetooth device.
       See instructions below to erase real nvram if the kit includes it. Also if the kit has nvram, set
       the download option to avoid writing to nvram with DIRECT_LOAD=1 in the makefile.
    2. Build the app with the nvram emulation library and NVRAM_EMULATION_HCI=1 in the makefile.
    3. Program the app to the kit.
    4. Run the script to restore any previous nvram if applicable. If any nvram items are restored the script
       will show "sent nvram backup restore" in the script console output
    5. Operate the app, for example by pairing with another Bluetooth device.
    6. Observe nvram updates in the script console output, such as "got nvram data update 512".
    7. Simulate a power failure by quitting the script (Ctrl-c) and removing power from the kit.
    8. Restore power to the kit and resume the application by Recovery Reset and programming the app to the kit.
    9. Run the script to restore the nvram state previous to the power removal.
    10. Resume operation with the peer device. If pairing keys, for example, were stored in nvram then they will
        be available to connect to the peer without re-pairing.

    Note that a reset or power off/on will clear the RAM and the application will need to be reprogrammed.
    If a kit includes FLASH and a program is in FLASH, it will be run if the device is reset. If needed,
    one way to clear out the FLASH is to modify the RAM download command line in the file ChipLoad.pl so that
    the FLASH is erased (shown below). This will cause the minidriver to be loaded and it will perform the
    chip erase, but the minidriver will interfere with the application in RAM. After this completes and the
    FLASH is erased, restore the original command line in the script and program the device again to load the
    application into RAM.

    if($direct_load == 1) {

      #print "$chip_load -BLUETOOLMODE -PORT $com_port -BAUDRATE $detected_baud -BTP $btp_file -NOERASE -CONFIG $config_file ${addl_flags}\n";
      - qx{"$chip_load" -BLUETOOLMODE -PORT $com_port -BAUDRATE $detected_baud -BTP "${btp_file}" -NOERASE -CONFIG ${config_file} -LOGTO "$download_log_path" ${addl_flags}};
      + qx{"$chip_load" -BLUETOOLMODE -PORT $com_port -BAUDRATE $detected_baud -MINIDRIVER "${mini_driver}" -BTP "${btp_file}" -CONFIG ${config_file} -LOGTO "$download_log_path" ${addl_flags}};
    }

    Note the default command line was modified to remove "-NOERASE" and add "-MINIDRIVER".
