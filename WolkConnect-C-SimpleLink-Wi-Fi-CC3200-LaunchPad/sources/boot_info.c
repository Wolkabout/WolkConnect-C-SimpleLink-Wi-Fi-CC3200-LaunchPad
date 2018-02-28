#include "boot_info.h"

_i32 WriteBootInfo(sBootInfo_t *psBootInfo)
{
    _i32 lFileHandle;
    _u32 ulToken;
    _i32 status = -1;

    if( 0 == sl_FsOpen((_u8 *)IMG_BOOT_INFO, FS_MODE_OPEN_WRITE, &ulToken, &lFileHandle) )
    {
        if( 0 < sl_FsWrite(lFileHandle, 0, (_u8 *)psBootInfo, sizeof(sBootInfo_t)) )
        {
            Report("WriteBootInfo: ucActiveImg=%d, ulImgStatus=0x%x\n\r", psBootInfo->ucActiveImg, psBootInfo->ulImgStatus);
            status = 0;
        }
        sl_FsClose(lFileHandle, 0, 0, 0);
    }

    return status;
}

_i32 ReadBootInfo(sBootInfo_t *psBootInfo)
{
    _i32 lFileHandle;
    _u32 ulToken;
    _i32 status = -1;

    if( 0 == sl_FsOpen((_u8 *)IMG_BOOT_INFO, FS_MODE_OPEN_READ, &ulToken, &lFileHandle) )
    {
        if( 0 < sl_FsRead(lFileHandle, 0, (_u8 *)psBootInfo, sizeof(sBootInfo_t)) )
        {
            status = 0;
            Report("ReadBootInfo: ucActiveImg=%d, ulImgStatus=0x%x\n\r", psBootInfo->ucActiveImg, psBootInfo->ulImgStatus);
        }
        sl_FsClose(lFileHandle, 0, 0, 0);
    }

    return status;
}

