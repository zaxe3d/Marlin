/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#include "../../FATFS/App/fatfs.h"
//#include "../../USB_HOST/App/usb_host.h"
#include <stdio.h>
#include <math.h>
//#include "../../sd/cardreader.h"

extern char fname2[/*20*/100];
char buf_main[50][200];  //DGUS
int file_cnt2=0; //DGUS
int file_cnt=0;
FATFS USBDISKFatFs;           /* File system object for USB disk logical drive */
FIL MyFile;                   /* File object */
char USBDISKPath[4];          /* USB Host logical drive path */
static void MSC_Application(void);
static void MSC_Application2(void);
static void MSC_Application4(void);
FRESULT Explore_Disk(char *path, uint8_t recu_level);
FRESULT Explore_Disk2(char *path, uint8_t recu_level);
FRESULT Explore_Disk3(char *path, uint8_t recu_level);
void usb_ls(void);
void prnt_els2(char * str); //Elsan for c++ files.
static void MSC_Application3(void);
static void MSC_Application5(void);

USBH_HandleTypeDef hUSBHost; /* USB Host handle */
typedef enum {
  APPLICATION_IDLE = 0,
  APPLICATION_START,
  APPLICATION_RUNNING,
}MSC_ApplicationTypeDef;
MSC_ApplicationTypeDef Appli_state = APPLICATION_IDLE;

#if ENABLED(SDSUPPORT)

#include "../gcode.h"
#include "../../sd/cardreader.h"

/**
 * M20: List SD card to serial output
 */
/*
void GcodeSuite::M20() {
  if (card.flag.mounted) {
    SERIAL_ECHOLNPGM(STR_BEGIN_FILE_LIST);
    card.ls();
    SERIAL_ECHOLNPGM(STR_END_FILE_LIST);
  }
  else
    SERIAL_ECHO_MSG(STR_NO_MEDIA);
}
*/
#endif // SDSUPPORT

static void MX_GPIO_Init(void)
{ //Elsan this part can be put in init functions of main.
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
}

static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id)
{
  switch(id)
  {
  case HOST_USER_SELECT_CONFIGURATION:
    break;

  case HOST_USER_DISCONNECTION:
    Appli_state = APPLICATION_IDLE;

    f_mount(NULL, (TCHAR const*)"", 0);
    break;

  case HOST_USER_CLASS_ACTIVE:
    Appli_state = APPLICATION_START;
    break;

  default:
    break;
  }
}

int FATFS_second=0;
void usb_ls(void)
{  
	char stre[200];
  long cntr=0;
    
  //SERIAL_ECHOLNPGM("usb_ls starting");
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  //HAL_Init();
  /* Configure the system clock */
  //SystemClock_Config();

  /* Initialize all configured peripherals */
  //Elsan this part can be put in init functions of main.
  //MX_GPIO_Init();   //Put in main setup.
  //MX_FATFS_Init();
  //MX_USB_HOST_Init();
    
  if(FATFS_second) {
    Appli_state = APPLICATION_IDLE;
    FATFS_UnLinkDriver(USBDISKPath);  //Elsan actually done when leaving MSC_Application.
    USBH_Stop(&hUSBHost);
    USBH_DeInit(&hUSBHost);    
  } 
    
  //##-1- Link the USB Host disk I/O driver ##################################
    if(FATFS_LinkDriver(&USBH_Driver, USBDISKPath) == 0)
    { 
      //SERIAL_ECHOLNPGM("Link the USB Host disk I/O driver");
      FATFS_second=1;
      
      //##-2- Init Host Library ################################################
      USBH_Init(&hUSBHost, USBH_UserProcess, 0); //Enable one by one.
      
      //##-3- Add Supported Class ##############################################
      USBH_RegisterClass(&hUSBHost, USBH_MSC_CLASS); //Enable one by one.
      
      //##-4- Start Host Process ###############################################
      USBH_Start(&hUSBHost); //Enable one by one.

      //##-5- Run Application (Blocking mode) ##################################
      //while (1) { //Elsan run one time as Marlin could crash.
      while(cntr<100000) {
        /*
        stre[0]=0;//clean content
        sprintf(stre, "%d", hUSBHost.gState);
        SERIAL_ECHOPGM("gState: ");
      	SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");
        */

        /*
        stre[0]=0;//clean content
        sprintf(stre, "%d", hUSBHost.device.is_connected);
        SERIAL_ECHOPGM("device.is_connected: ");
      	SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");
        */

        // USB Host Background task 
        USBH_Process(&hUSBHost);

        /*
        stre[0]=0;//clean content
        sprintf(stre, "%d", hUSBHost.gState);
        SERIAL_ECHOPGM("gState: ");
      	SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");
        */
        /*
        stre[0]=0;//clean content
        sprintf(stre, "%d", Appli_state);
        SERIAL_ECHOPGM("Appli_state: ");
      	SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");
        */

        // Mass Storage Application State Machine 
        switch(Appli_state)
        {
        case APPLICATION_START:
        
      	MSC_Application();
        
        stre[0]=0;//clean content
      	//sprintf(stre, "%d", hUSBHost.device.DevDesc.idVendor);
      	//SERIAL_ECHOPGM("Vendor ID: ");
      	//SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");

      	stre[0]=0;//clean content
      	//sprintf(stre, "%d", hUSBHost.device.speed);
      	//SERIAL_ECHOPGM("Speed (0-HS, 1-FS, 2-LS): ");
      	//SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");

        Appli_state = APPLICATION_IDLE;
        //SERIAL_ECHOLNPGM("usb_ls leave");
        return; //Elsan no need to wait for remaining loops.
        break;

        case APPLICATION_IDLE:
        default:
        break;
        }

       cntr++; 
      }
    }
    
  //SERIAL_ECHOLNPGM("usb_ls finished");
}

//For DGUS
void usb_ls2(void)
{  
	char stre[200];
  long cntr=0;
      
  //SERIAL_ECHOLNPGM("usb_ls starting");  
  /* Initialize all configured peripherals */
  //Elsan this part can be put in init functions of main.
  //MX_GPIO_Init(); //Put in main setup.
  //MX_FATFS_Init();
  //MX_USB_HOST_Init();
      
  if(FATFS_second) {
    Appli_state = APPLICATION_IDLE;
    FATFS_UnLinkDriver(USBDISKPath);  //Elsan actually done when leaving MSC_Application.
    USBH_Stop(&hUSBHost);
    USBH_DeInit(&hUSBHost);        
  } 
    
  //##-1- Link the USB Host disk I/O driver ##################################
    if(FATFS_LinkDriver(&USBH_Driver, USBDISKPath) == 0)
    { 
      //SERIAL_ECHOLNPGM("Link the USB Host disk I/O driver");
      FATFS_second=1;
      
      //##-2- Init Host Library ################################################
      USBH_Init(&hUSBHost, USBH_UserProcess, 0); 
      
      //##-3- Add Supported Class ##############################################
      USBH_RegisterClass(&hUSBHost, USBH_MSC_CLASS); 
      
      //##-4- Start Host Process ###############################################
      USBH_Start(&hUSBHost); //Enable one by one.

      //##-5- Run Application (Blocking mode) ##################################
      //while (1) { //Elsan run one time as Marlin could crash.
      while(cntr<100000) { //100000
        /*
        stre[0]=0;//clean content
        sprintf(stre, "%d", hUSBHost.gState);
        SERIAL_ECHOPGM("gState: ");
      	SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");
        */

        /*
        stre[0]=0;//clean content
        sprintf(stre, "%d", hUSBHost.device.is_connected);
        SERIAL_ECHOPGM("device.is_connected: ");
      	SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");
        */

        // USB Host Background task 
        USBH_Process(&hUSBHost);

        /*
        stre[0]=0;//clean content
        sprintf(stre, "%d", hUSBHost.gState);
        SERIAL_ECHOPGM("gState: ");
      	SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");
        */
        /*
        stre[0]=0;//clean content
        sprintf(stre, "%d", Appli_state);
        SERIAL_ECHOPGM("Appli_state: ");
      	SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");
        */

        // Mass Storage Application State Machine 
        switch(Appli_state)
        {
        case APPLICATION_START:
        
      	MSC_Application4();
        
        stre[0]=0;//clean content
      	//sprintf(stre, "%d", hUSBHost.device.DevDesc.idVendor);
      	//SERIAL_ECHOPGM("Vendor ID: ");
      	//SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");

      	stre[0]=0;//clean content
      	//sprintf(stre, "%d", hUSBHost.device.speed);
      	//SERIAL_ECHOPGM("Speed (0-HS, 1-FS, 2-LS): ");
      	//SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");

        Appli_state = APPLICATION_IDLE;
        //SERIAL_ECHOLNPGM("usb_ls leave");
        return; //Elsan no need to wait for remaining loops.
        break;

        case APPLICATION_IDLE:
        default:
        break;
        }

       cntr++; 
      }
    }
    
  //SERIAL_ECHOLNPGM("usb_ls finished");
}

static void MSC_Application(void)
{
  FRESULT res;                                          /* FatFs function common result code */
  int err_code;/* File read buffer */
  
  /* Register the file system object to the FatFs module */
  if(f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0) != FR_OK)
  {
    /* FatFs Initialization Error */
    SERIAL_ECHOPGM("MSC_App->f_mount error");
    Error_Handler();
  }
  else
  {	    
    SERIAL_ECHOLNPGM(STR_BEGIN_FILE_LIST);
    Explore_Disk2("0:/", 1);
    SERIAL_ECHOLNPGM(STR_END_FILE_LIST);
    
  }

  /* Unlink the USB disk I/O driver */
  FATFS_UnLinkDriver(USBDISKPath);
}

static void MSC_Application4(void)
{  
  /* Register the file system object to the FatFs module */
  if(f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0) != FR_OK)
  {
    /* FatFs Initialization Error */
    SERIAL_ECHOPGM("MSC_App->f_mount error");
    Error_Handler();
  }
  else
  {	
    Explore_Disk3("0:/", 1);  
  }

  /* Unlink the USB disk I/O driver */
  FATFS_UnLinkDriver(USBDISKPath);
}

void prnt_els2(char * str);

//Elsan
void GcodeSuite::M20() {
  //if (card.flag.mounted) {
    //SERIAL_ECHOLNPGM(STR_BEGIN_FILE_LIST);
    //prnt_els2("GcodeSuite::M20");  
    //card.ls();
    usb_ls();
    //SERIAL_ECHOLNPGM(STR_END_FILE_LIST);
  //}
  //else
  //  SERIAL_ECHO_MSG(STR_NO_MEDIA);
}

//Elsan for directory tree structure demo. Not used.
FRESULT Explore_Disk(char *path, uint8_t recu_level)
{
  FRESULT res = FR_OK;
  FILINFO fno;
  DIR dir;
  char *fn;
  char tmp[14];
  //uint8_t line_idx = 0;
  char stre[20];

  res = f_opendir(&dir, path);

  if(res == FR_OK)
  {

    while(USBH_MSC_IsReady(&hUSBHost))
    {
      res = f_readdir(&dir, &fno);
      if(res != FR_OK || fno.fname[0] == 0)
      {
        break;
      }
      if(fno.fname[0] == '.')
      {
        continue;
      }

      fn = fno.fname;
      strcpy(tmp, fn);

      //line_idx++;
      //if(line_idx > 9)
      //{
      //  line_idx = 0;
      //  LCD_UsrLog("> Press User button \n");
      //  LCD_UsrLog("> To Continue.\n");

        /* KEY Button in polling */
      //  while((BSP_PB_GetState(BUTTON_USER) != SET) && (Appli_state != APPLICATION_DISCONNECT))
      //  {
          /* Wait for User Input */
      //  }
      //}

      if(recu_level == 1)
      {
        //LCD_DbgLog("   |__");
    	  //prnt2("   |__",6);
        SERIAL_ECHOPGM("   |__");
      }
      else if(recu_level == 2)
      {
        //LCD_DbgLog("   |   |__");
    	  //prnt2("   |   |__",10);
        SERIAL_ECHOPGM("   |   |__");
      }

      if((fno.fattrib & AM_DIR) == AM_DIR)      
      {
        //LCD_UsrLog((void *)tmp);
    	  //prnt(tmp,strlen(tmp));
        SERIAL_ECHOPGM(tmp);SERIAL_ECHOLNPGM("");
        Explore_Disk(fn, 2);
      }
      else
      {
        //LCD_DbgLog((void *)tmp);
        //prnt2(tmp,strlen(tmp));
        SERIAL_ECHOPGM(tmp);
        //prnt2("   size:",8);
        SERIAL_ECHOPGM("   size:");
        sprintf(stre, "%ld", fno.fsize);
        //prnt(stre,strlen(stre));
        SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");
      }

      if(((fno.fattrib & AM_DIR) == AM_DIR)&&(recu_level == 2))      
      {
        Explore_Disk(fn, 2);
      }
    }
    f_closedir(&dir);
  }
  //LCD_UsrLog("> Select an operation to Continue.\n" );
  
  return res;
}


char path_prev[100];

FRESULT Explore_Disk2(char *path, uint8_t recu_level)
{
  FRESULT res = FR_OK;
  FILINFO fno;
  DIR dir;
  char *fn;
  char tmp[/*14*/100];
  //uint8_t line_idx = 0;
  char stre[/*100*/40]; //Elsan 100 does not work in LFN.
  char path2[100];
  
  res = f_opendir(&dir, path);

  if(res == FR_OK)
  {

    while(USBH_MSC_IsReady(&hUSBHost))
    {
      res = f_readdir(&dir, &fno);
      if(res != FR_OK || fno.fname[0] == 0)
      {
        break;
      }
      //if(fno.fname[0] == '.')
      if((fno.fname[0] == '.')||((fno.fattrib & AM_HID) == AM_HID)) //Elsan test
      {
        continue;
      }

      fn = fno.fname;
      strcpy(tmp, fn);

      strcpy(path2,""); //Elsan

      if(recu_level == 1)
      {
        //prnt2("   |__",6);
        //SERIAL_ECHOPGM("   |__");        
      }
      else if(recu_level == 2)
      {
        //prnt2("   |   |__",10);
        //SERIAL_ECHOPGM("   |   |__");        
      }

      //if((fno.fattrib & AM_DIR) == AM_DIR)
      if(((fno.fattrib & AM_DIR) == AM_DIR)&&((fno.fattrib & AM_HID) != AM_HID))
      {
        //prnt(tmp,strlen(tmp));
        //SERIAL_ECHOPGM(tmp);SERIAL_ECHOLNPGM(""); 

        strcpy(path2,"/");
        strcat(path2,tmp); strcat(path2,"/");
        if(recu_level==1) strcpy(path_prev,path2);
        Explore_Disk2(fn, 2);
      }
      else
      {
        //prnt2(tmp,strlen(tmp));

        if(recu_level==1) SERIAL_ECHOPGM(path2);
        else SERIAL_ECHOPGM(path_prev);        

        SERIAL_ECHOPGM(tmp); 
        SERIAL_ECHOPGM(" ");

        //prnt2("   size:",8);
        //SERIAL_ECHOPGM("   size:");
        sprintf(stre, "%ld", fno.fsize);        
        SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");
      }

      //if(((fno.fattrib & AM_DIR) == AM_DIR)&&(recu_level == 2))
      if(((fno.fattrib & AM_DIR) == AM_DIR)&&((fno.fattrib & AM_HID) != AM_HID)&&(recu_level == 2))
      {
        Explore_Disk2(fn, 2);
      }
    }
    f_closedir(&dir);
  }
  //LCD_UsrLog("> Select an operation to Continue.\n" );
  
  return res;
}


FRESULT Explore_Disk3(char *path, uint8_t recu_level)
{
  FRESULT res = FR_OK;
  FILINFO fno;
  DIR dir;
  char *fn;
  char tmp[/*14*/100];
  //uint8_t line_idx = 0;
  //char stre[100];
  char path2[100];  
    
  res = f_opendir(&dir, path);

  if(res == FR_OK)
  {
    while(USBH_MSC_IsReady(&hUSBHost))
    {
      res = f_readdir(&dir, &fno);
      if(res != FR_OK || fno.fname[0] == 0)
      {
        break;
      }
      //if(fno.fname[0] == '.')
      if((fno.fname[0] == '.')||((fno.fattrib & AM_HID) == AM_HID)) //Elsan test
      {
        continue;
      }

      fn = fno.fname;
      strcpy(tmp, fn);

      strcpy(path2,""); //Elsan

      if(recu_level == 1)
      {
        //LCD_DbgLog("   |__");
    	  //prnt2("   |__",6);
        //SERIAL_ECHOPGM("   |__");

        //strcpy(path2,"");
        //strcpy(path2,"/");
        //strcat(path2,tmp); strcat(path2,"/");
      }
      else if(recu_level == 2)
      {
        //LCD_DbgLog("   |   |__");
    	  //prnt2("   |   |__",10);
        //SERIAL_ECHOPGM("   |   |__");

        //strcpy(path2,"/");
        //strcat(path2,"/");
        //strcat(path2,tmp); strcat(path2,"/");
        //SERIAL_ECHOPGM(path_prev);
      }

      //if((fno.fattrib & AM_DIR) == AM_DIR)
      if(((fno.fattrib & AM_DIR) == AM_DIR)&&((fno.fattrib & AM_HID) != AM_HID))
      {
        //LCD_UsrLog((void *)tmp);
    	  //SERIAL_ECHOPGM(tmp);SERIAL_ECHOLNPGM("");

        strcpy(path2,"/");
        strcat(path2,tmp); strcat(path2,"/");
        if(recu_level==1) strcpy(path_prev,path2);
        //SERIAL_ECHOPGM(path2);
        //if(recu_level==1) SERIAL_ECHOPGM(path2);
        //else SERIAL_ECHOPGM(path_prev);
        Explore_Disk3(fn, 2);
      }
      else
      {        
        if(recu_level==1) {strcpy(buf_main[file_cnt],path2);}
        else {strcpy(buf_main[file_cnt], path_prev);}
                
        strcat(buf_main[file_cnt], tmp); 
        //int a=strlen(tmp); //dummy
        //strcat(buf_main[file_cnt], "\r\n");
        //buf_main[file_cnt][31]=10;
        //SERIAL_ECHOPGM("qq");SERIAL_ECHOLNPGM(""); 
        //SERIAL_ECHOPGM(buf_main[file_cnt]);SERIAL_ECHOLNPGM("");               
        file_cnt++;
        file_cnt2++;
                       
      }

      //if(((fno.fattrib & AM_DIR) == AM_DIR)&&(recu_level == 2))
      if(((fno.fattrib & AM_DIR) == AM_DIR)&&((fno.fattrib & AM_HID) != AM_HID)&&(recu_level == 2))
      {
        Explore_Disk3(fn, 2);
      }
    }
    f_closedir(&dir);
  }
  //LCD_UsrLog("> Select an operation to Continue.\n" );
    
  return res;
}


void usb_file_open(void)
{  
	char stre[/*200*/100];
  long cntr=0;
    
  //SERIAL_ECHOLNPGM("usb_ls starting");
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  //HAL_Init();
  /* Configure the system clock */
  //SystemClock_Config();

  /* Initialize all configured peripherals */
  //Elsan this part can be put in init functions of main.
  //MX_GPIO_Init(); //Put in main setup.
  //MX_FATFS_Init();
  //MX_USB_HOST_Init();
    
  if(FATFS_second) {
    Appli_state = APPLICATION_IDLE;
    FATFS_UnLinkDriver(USBDISKPath);  //Elsan actually done when leaving MSC_Application.
    USBH_Stop(&hUSBHost);
    USBH_DeInit(&hUSBHost);    
  } 
    
  //##-1- Link the USB Host disk I/O driver ##################################
    if(FATFS_LinkDriver(&USBH_Driver, USBDISKPath) == 0)
    { 
      //SERIAL_ECHOLNPGM("Link the USB Host disk I/O driver");
      FATFS_second=1;
      
      //##-2- Init Host Library ################################################
      USBH_Init(&hUSBHost, USBH_UserProcess, 0); //Enable one by one.
      
      //##-3- Add Supported Class ##############################################
      USBH_RegisterClass(&hUSBHost, USBH_MSC_CLASS); //Enable one by one.
      
      //##-4- Start Host Process ###############################################
      USBH_Start(&hUSBHost); //Enable one by one.

      //##-5- Run Application (Blocking mode) ##################################
      //while (1) { //Elsan run one time as Marlin could crash.
      while(cntr<100000) {
        /*
        stre[0]=0;//clean content
        sprintf(stre, "%d", hUSBHost.gState);
        SERIAL_ECHOPGM("gState: ");
      	SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");
        */

        /*
        stre[0]=0;//clean content
        sprintf(stre, "%d", hUSBHost.device.is_connected);
        SERIAL_ECHOPGM("device.is_connected: ");
      	SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");
        */

        // USB Host Background task 
        USBH_Process(&hUSBHost);

        /*
        stre[0]=0;//clean content
        sprintf(stre, "%d", hUSBHost.gState);
        SERIAL_ECHOPGM("gState: ");
      	SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");
        */
        /*
        stre[0]=0;//clean content
        sprintf(stre, "%d", Appli_state);
        SERIAL_ECHOPGM("Appli_state: ");
      	SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");
        */

        // Mass Storage Application State Machine 
        switch(Appli_state)
        {
        case APPLICATION_START:
        
      	MSC_Application2();
        
        stre[0]=0;//clean content
      	//sprintf(stre, "%d", hUSBHost.device.DevDesc.idVendor);
      	//SERIAL_ECHOPGM("Vendor ID: ");
      	//SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");

      	stre[0]=0;//clean content
      	//sprintf(stre, "%d", hUSBHost.device.speed);
      	//SERIAL_ECHOPGM("Speed (0-HS, 1-FS, 2-LS): ");
      	//SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");

        Appli_state = APPLICATION_IDLE;
        //SERIAL_ECHOLNPGM("usb_ls leave");
        return; //Elsan no need to wait for remaining loops.
        break;

        case APPLICATION_IDLE:
        default:
        break;
        }

       cntr++; 
      }
    }
    
  //SERIAL_ECHOLNPGM("usb_ls finished");
}

void usb_file_open_wr(void)
{  
	char stre[200];
  long cntr=0;
    
  //SERIAL_ECHOLNPGM("usb_ls starting");
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  //HAL_Init();
  /* Configure the system clock */
  //SystemClock_Config();

  /* Initialize all configured peripherals */
  //Elsan this part can be put in init functions of main.
  //MX_GPIO_Init(); //Put in main setup.
  //MX_FATFS_Init();
  //MX_USB_HOST_Init();
    
  if(FATFS_second) {
    Appli_state = APPLICATION_IDLE;
    FATFS_UnLinkDriver(USBDISKPath);  //Elsan actually done when leaving MSC_Application.
    USBH_Stop(&hUSBHost);
    USBH_DeInit(&hUSBHost);    
  } 
    
  //##-1- Link the USB Host disk I/O driver ##################################
    if(FATFS_LinkDriver(&USBH_Driver, USBDISKPath) == 0)
    { 
      //SERIAL_ECHOLNPGM("Link the USB Host disk I/O driver");
      FATFS_second=1;
      
      //##-2- Init Host Library ################################################
      USBH_Init(&hUSBHost, USBH_UserProcess, 0); //Enable one by one.
      
      //##-3- Add Supported Class ##############################################
      USBH_RegisterClass(&hUSBHost, USBH_MSC_CLASS); //Enable one by one.
      
      //##-4- Start Host Process ###############################################
      USBH_Start(&hUSBHost); //Enable one by one.

      //##-5- Run Application (Blocking mode) ##################################
      //while (1) { //Elsan run one time as Marlin could crash.
      while(cntr<100000) {
        /*
        stre[0]=0;//clean content
        sprintf(stre, "%d", hUSBHost.gState);
        SERIAL_ECHOPGM("gState: ");
      	SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");
        */

        /*
        stre[0]=0;//clean content
        sprintf(stre, "%d", hUSBHost.device.is_connected);
        SERIAL_ECHOPGM("device.is_connected: ");
      	SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");
        */

        // USB Host Background task 
        USBH_Process(&hUSBHost);

        /*
        stre[0]=0;//clean content
        sprintf(stre, "%d", hUSBHost.gState);
        SERIAL_ECHOPGM("gState: ");
      	SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");
        */
        /*
        stre[0]=0;//clean content
        sprintf(stre, "%d", Appli_state);
        SERIAL_ECHOPGM("Appli_state: ");
      	SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");
        */

        // Mass Storage Application State Machine 
        switch(Appli_state)
        {
        case APPLICATION_START:
        
      	MSC_Application3();
        
        stre[0]=0;//clean content
      	sprintf(stre, "%d", hUSBHost.device.DevDesc.idVendor);
      	//SERIAL_ECHOPGM("Vendor ID: ");
      	//SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");

      	stre[0]=0;//clean content
      	sprintf(stre, "%d", hUSBHost.device.speed);
      	//SERIAL_ECHOPGM("Speed (0-HS, 1-FS, 2-LS): ");
      	//SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");

        Appli_state = APPLICATION_IDLE;
        //SERIAL_ECHOLNPGM("usb_ls leave");
        return; //Elsan no need to wait for remaining loops.
        break;

        case APPLICATION_IDLE:
        default:
        break;
        }

       cntr++; 
      }
    }
    
  //SERIAL_ECHOLNPGM("usb_ls finished");
}

void usb_file_del(void)
{  
	char stre[200];
  long cntr=0;
    
  //SERIAL_ECHOLNPGM("usb_ls starting");
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  //HAL_Init();
  /* Configure the system clock */
  //SystemClock_Config();

  /* Initialize all configured peripherals */
  //Elsan this part can be put in init functions of main.
  //MX_GPIO_Init(); //Put in main setup.
  //MX_FATFS_Init();
  //MX_USB_HOST_Init();
    
  if(FATFS_second) {
    Appli_state = APPLICATION_IDLE;
    FATFS_UnLinkDriver(USBDISKPath);  //Elsan actually done when leaving MSC_Application.
    USBH_Stop(&hUSBHost);
    USBH_DeInit(&hUSBHost);    
  } 
    
  //##-1- Link the USB Host disk I/O driver ##################################
    if(FATFS_LinkDriver(&USBH_Driver, USBDISKPath) == 0)
    { 
      //SERIAL_ECHOLNPGM("Link the USB Host disk I/O driver");
      FATFS_second=1;
      
      //##-2- Init Host Library ################################################
      USBH_Init(&hUSBHost, USBH_UserProcess, 0); //Enable one by one.
      
      //##-3- Add Supported Class ##############################################
      USBH_RegisterClass(&hUSBHost, USBH_MSC_CLASS); //Enable one by one.
      
      //##-4- Start Host Process ###############################################
      USBH_Start(&hUSBHost); //Enable one by one.

      //##-5- Run Application (Blocking mode) ##################################
      //while (1) { //Elsan run one time as Marlin could crash.
      while(cntr<100000) {
        /*
        stre[0]=0;//clean content
        sprintf(stre, "%d", hUSBHost.gState);
        SERIAL_ECHOPGM("gState: ");
      	SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");
        */

        /*
        stre[0]=0;//clean content
        sprintf(stre, "%d", hUSBHost.device.is_connected);
        SERIAL_ECHOPGM("device.is_connected: ");
      	SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");
        */

        // USB Host Background task 
        USBH_Process(&hUSBHost);

        /*
        stre[0]=0;//clean content
        sprintf(stre, "%d", hUSBHost.gState);
        SERIAL_ECHOPGM("gState: ");
      	SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");
        */
        /*
        stre[0]=0;//clean content
        sprintf(stre, "%d", Appli_state);
        SERIAL_ECHOPGM("Appli_state: ");
      	SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");
        */

        // Mass Storage Application State Machine 
        switch(Appli_state)
        {
        case APPLICATION_START:
        
      	MSC_Application5();
        
        stre[0]=0;//clean content
      	sprintf(stre, "%d", hUSBHost.device.DevDesc.idVendor);
      	//SERIAL_ECHOPGM("Vendor ID: ");
      	//SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");

      	stre[0]=0;//clean content
      	sprintf(stre, "%d", hUSBHost.device.speed);
      	//SERIAL_ECHOPGM("Speed (0-HS, 1-FS, 2-LS): ");
      	//SERIAL_ECHOPGM(stre);SERIAL_ECHOLNPGM("");

        Appli_state = APPLICATION_IDLE;
        //SERIAL_ECHOLNPGM("usb_ls leave");
        return; //Elsan no need to wait for remaining loops.
        break;

        case APPLICATION_IDLE:
        default:
        break;
        }

       cntr++; 
      }
    }
    
  //SERIAL_ECHOLNPGM("usb_ls finished");
}

static void MSC_Application2(void)
{
  FRESULT res;  /* FatFs function common result code */
  int err_code; /* File read buffer */
  
  /* Register the file system object to the FatFs module */
  if(f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0) != FR_OK)
  {
    /* FatFs Initialization Error */
    SERIAL_ECHOPGM("MSC_App->f_mount error");
    Error_Handler();
  }
  else
  {	    
    /* Open a new text file object with write access */         
    err_code=f_open(&MyFile, /*card.filename*/(const char*) &fname2[0], FA_READ);
    
    if(err_code!=FR_OK)
    {      
      SERIAL_ECHOPGM("MSC_App->f_open:error");
      //sprintf(stre, "%d", err_code);      
      Error_Handler();
    }
    else
    { 
      //SERIAL_ECHOLNPGM("MSC_App->f_open");
      return; //Elsan only f_open is needed.      
    }
  }

  /* Unlink the USB disk I/O driver */
  FATFS_UnLinkDriver(USBDISKPath);
}


static void MSC_Application3(void)
{
  FRESULT res;                                          /* FatFs function common result code */
  int err_code;/* File read buffer */
  
  /* Register the file system object to the FatFs module */
  if(f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0) != FR_OK)
  {
    /* FatFs Initialization Error */
    SERIAL_ECHOPGM("MSC_App->f_mount error");
    Error_Handler();
  }
  else
  {	
    /* Create and Open a new text file object with write access */    
    err_code=f_open(&MyFile, /*card.filename*/(const char*) &fname2[0], FA_CREATE_ALWAYS | FA_WRITE);
    
    if(err_code!=FR_OK)
    {
      SERIAL_ECHOPGM("MSC_App->f_open:error");
      //sprintf(stre, "%d", err_code);      
      Error_Handler();
    }
    else
    {       
      return; //Elsan only f_open is needed.
    }
  }

  /* Unlink the USB disk I/O driver */
  FATFS_UnLinkDriver(USBDISKPath);
}

static void MSC_Application5(void)
{
  FRESULT res;                                          /* FatFs function common result code */
  int err_code;/* File read buffer */
    
  /* Register the file system object to the FatFs module */
  if(f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0) != FR_OK)
  {
    /* FatFs Initialization Error */
    SERIAL_ECHOPGM("MSC_App->f_mount error");
    Error_Handler();
  }
  else
  {	    
    /* Delete */
    err_code=f_unlink((const char*) &fname2[0]);
    
    if(err_code!=FR_OK)
    {
      /* Error */
      SERIAL_ECHOPGM("MSC_App->f_open:error");
      //sprintf(stre, "%d", err_code);      
      Error_Handler();
    }
    else
    {       
      return; //Elsan only f_open is needed.    
    }
  }

  /* Unlink the USB disk I/O driver */
  FATFS_UnLinkDriver(USBDISKPath);
}