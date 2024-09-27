## Default Wiced HCI Configuration parameters
##
## These can be overriden by using the following optional command line parameters:
##       -com n             Use COM Port 'n'
##       -baud n            Use Baud Rate 'n'
##       -inst n            Trace to Spy Instance 'n'
##       -log filePath      Log to file 'filePath'
##
DEFAULT_COMPORT         = 19             # Set to 0 to use TCP via TCP2COM
DEFAULT_BAUDRATE        = 3000000
DEFAULT_SCRIPT_INSTANCE = 0              # Used for Spy and TCP2COM instances
DEFAULT_CHECK_CTS       = 1              # Set to 1 for chips (like 20719) that need us to
                                         # check for CTS toggle while resetting
