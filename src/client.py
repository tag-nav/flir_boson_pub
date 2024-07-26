# import ClientFiles_Python.Client_API
# import ClientFiles_Python.Client_Packager
# import ClientFiles_Python.Serializer_Struct

from BosonSDK import *

def set_and_get_ext_sync_mode():
    EXT_SYNC_MODE = FLR_BOSON_EXT_SYNC_MODE_E.FLR_BOSON_EXT_SYNC_DISABLE_MODE
    myCam = CamAPI.pyClient(manualport="/dev/ttyACM0", manualbaud=921600) # or manualport="COM7" on Windows
    returncode = myCam.bosonSetExtSyncMode(EXT_SYNC_MODE)
    result, mode = myCam.bosonGetExtSyncMode()
    return result, mode

if __name__ == "__main__":
    result, mode = set_and_get_ext_sync_mode()
    print(f"External sync mode set to: {mode}")
