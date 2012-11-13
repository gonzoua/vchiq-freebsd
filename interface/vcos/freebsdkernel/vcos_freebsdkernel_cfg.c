/*****************************************************************************
* Copyright 2009 - 2010 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/

#include "interface/vcos/vcos.h"

MALLOC_DEFINE(M_VCCFG, "vccfg", "VideoCore config subsystem memory");

struct opaque_vcos_cfg_buf_t
{
    struct seq_file *seq;
    char            *charBuf;
};

struct opaque_vcos_cfg_entry_t
{
    /* TODO: add sysctl stuff */
    VCOS_CFG_SHOW_FPTR     showFunc;
    VCOS_CFG_PARSE_FPTR    parseFunc;
    void                  *data;
    const char            *name;
};


/***************************************************************************** 
* 
*    vcos_cfg_mkdir
*  
*****************************************************************************/

VCOS_STATUS_T vcos_cfg_mkdir( VCOS_CFG_ENTRY_T *entryp,
                              VCOS_CFG_ENTRY_T *parent,
                              const char *dirName )
{
    VCOS_CFG_ENTRY_T    entry;

    if (( entry = malloc( sizeof( *entry ), M_VCCFG, M_WAITOK | M_ZERO )) == NULL )
    {
        return VCOS_ENOMEM;
    }

    printf("IMPLEMENT ME: vcos_cfg_mkdir %s\n", dirName);

    entry->name = dirName;

    *entryp = entry;
    return VCOS_SUCCESS;
}

/***************************************************************************** 
* 
*    vcos_cfg_create_entry
*  
*****************************************************************************/

VCOS_STATUS_T vcos_cfg_create_entry( VCOS_CFG_ENTRY_T *entryp,
                                     VCOS_CFG_ENTRY_T *parent,
                                     const char *entryName,
                                     VCOS_CFG_SHOW_FPTR showFunc,
                                     VCOS_CFG_PARSE_FPTR parseFunc,
                                     void *data )
{
    VCOS_CFG_ENTRY_T    entry;
    mode_t              mode;

    *entryp = NULL;

    printf("IMPLEMENT ME: vcos_cfg_create_entry %s\n", entryName);

    if (( entry = malloc( sizeof( *entry ), M_VCCFG, M_WAITOK | M_ZERO )) == NULL )
    {
        return VCOS_ENOMEM;
    }

    entry->showFunc = showFunc;
    entry->parseFunc = parseFunc;
    entry->data = data;
    entry->name = entryName;

    *entryp = entry;
    return VCOS_SUCCESS;    
}

/***************************************************************************** 
* 
*    vcos_cfg_remove_entry
*  
*****************************************************************************/

VCOS_STATUS_T vcos_cfg_remove_entry( VCOS_CFG_ENTRY_T *entryp )
{
    if (( entryp != NULL ) && ( *entryp != NULL ))
    {

        free( *entryp, M_VCCFG );
        *entryp = NULL;
    }

    return VCOS_SUCCESS;
}

/***************************************************************************** 
* 
*    vcos_cfg_is_entry_created
*  
*****************************************************************************/

int vcos_cfg_is_entry_created( VCOS_CFG_ENTRY_T entry )
{
    return ( entry != NULL );
}

/***************************************************************************** 
* 
*    vcos_cfg_buf_printf
*  
*****************************************************************************/

void vcos_cfg_buf_printf( VCOS_CFG_BUF_T buf, const char *fmt, ... )
{
    va_list args;

    va_start(args, fmt);
    printf("IMPLEMENT ME vcos_cfg_buf_printf\n");
    va_end(args);
}

/***************************************************************************** 
* 
*    vcos_cfg_buf_get_str
*  
*****************************************************************************/

char *vcos_cfg_buf_get_str( VCOS_CFG_BUF_T buf )
{
    return buf->charBuf;
}
