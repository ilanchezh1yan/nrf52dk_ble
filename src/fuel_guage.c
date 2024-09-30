#define F_G_
#include "./fuel_gauge.h"

#define SCL_PIN 27
#define SDA_PIN 26

#define SLAVE_ADDRESS 0x36
#define SLAVE_READ (SLAVE_ADDRESS << 1) | 1
#define SLAVE_WRITE (SLAVE_ADDRESS << 1)

/*
    structure for modelconfigure(0xDB) register of MAX1726x IC.
    
    ModelID: Choose from one of the following Lithium models. For most batteries, use ModelID = 0.
      • ModelID = 0: Use for most lithium cobalt-oxide variants (a large majority of lithium in the
                     market place). Supported by EZ without characterization.
      • ModelID = 2: Use for lithium NCR or NCA cells such as Panasonic. Supported by EZ without
                     characterization.
      • ModelID = 6: Use for lithium iron-phosphate (LiFePO4). For better performance, a custom
                     characterization is recommended in this case, instead of an EZ configuration.
    
    NOTE : The variables with const qualifiers must be always 0. Reserved variable is read only variable.
           ***According to thid assigning values to the const variable only shows warning. 
           So Don't take warning message  for granted.***

 */

struct ModelCFG_Reg {
      const uint8_t D0_D1 : 2;
      uint8_t Reserved : 2;
      uint8_t modelID : 4;
      const uint8_t D8_D9 : 2;
      uint8_t VChg : 1;
      const uint8_t D11_D12 : 2;
      uint8_t R100 : 1;
      const uint8_t D14 : 1;
      uint8_t Refresh : 1;   
};

enum Read_Reg_Address {
      IChgTerm = 0x1E,
      ModelCFG = 0xDB,
      DesignCap = 0x18,
      RepSOC = 0x06,
      RepCap = 0x05,
      FullCapRep = 0x10,
      Age = 0x07,
      Cycles = 0x17,
      TimerH = 0xBE,
      Rcell = 0x14,
      TTE = 0x11,
      TTF = 0x20,
      Rsence = 0xD0,
      Vempty = 0x3A,
};


extern char data_array[10];   //from main.c
extern char App_data_array[20]; // from main.c
static const nrf_drv_twi_t twi_master = NRF_DRV_TWI_INSTANCE(0);

void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t fuel_gauge_module = {
       .scl                = SCL_PIN,
       .sda                = SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&twi_master, &fuel_gauge_module, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&twi_master);
}

static uint8_t chksum8(const unsigned char *buff, size_t len)
{
    unsigned int sum;
    for ( sum = 0 ; len != 0 ; len-- )
        sum += *(buff++);
    return (uint8_t)sum;
}

static uint16_t Master_read(uint8_t Reg_add)
{
  ret_code_t err;
  uint16_t buffer = 0;

  err = nrf_drv_twi_tx(&twi_master, SLAVE_ADDRESS, &Reg_add, 1, false);
  if(err != NRF_SUCCESS)
    NRF_LOG_DEBUG("twi error");
  
  err = nrf_drv_twi_rx(&twi_master, SLAVE_ADDRESS, (uint8_t *)&buffer, 2);
  if(err != NRF_SUCCESS)
    NRF_LOG_DEBUG("twi error");

  return buffer;
}

static void Master_write(struct Write_chunk Reg)
{
  ret_code_t err;
  
  err = nrf_drv_twi_tx(&twi_master, SLAVE_ADDRESS, (uint8_t *)&Reg, 3, false);
  if(err != NRF_SUCCESS)
    NRF_LOG_DEBUG("twi error");
}

static void FG_Reg_write(uint8_t address, uint16_t data)
{
  struct Write_chunk Reg;

  //data = (data << 8) | (data >> 8);

  Reg.address = address;
  Reg.data = data;  
  Master_write(Reg);
}

void battery_status(void) 
{ 

  int16_t *ptr = (uint16_t *)&data_array[5];
  data_array[3] = (uint8_t)(Master_read(RepSOC) >> 8);

  if(data_array[4]) {
          *ptr = Master_read(TTE);
          data_array[4] |= 1 << 1;
           
  }
  else {
          *ptr = Master_read(TTF);
          data_array[4] &= ~(3 << 1);
  }

  data_array[9] = chksum8(&data_array[3], 6);
}

void battery_profile(void *data_pointer)
{
  struct fuel_gauge *Profile_data = (struct fuel_gauge *)data_pointer;
  Profile_data->FG.FullCapRep = Master_read(FullCapRep);
  Profile_data->FG.Age = (uint8_t)(Master_read(Age) >> 8);
  Profile_data->FG.Cycles = Master_read(Cycles);
  Profile_data->FG.TimerH = Master_read(TimerH);
  Profile_data->FG.RepCap = Master_read(RepCap);
  Profile_data->FG.Rcell = Master_read(Rcell);
  Profile_data->CRC = chksum8((const unsigned char *)&Profile_data->FG.FullCapRep, 11); 
}

void configure_battery()
{
  uint16_t *struct_ptr;

  struct ModelCFG_Reg Data = {
                          .D0_D1 = 0,
                          .D8_D9 = 0,
                          .D11_D12 = 0,
                          .R100 = 0,
                          .D14 = 0,
                          .Refresh = 0
                        };
                   
  Data.modelID = App_data_array[5];
  Data.VChg = App_data_array[8];
  struct_ptr = (uint16_t *)&Data;

  FG_Reg_write(ModelCFG, *struct_ptr);  // to configure battery model based on chemical property.

  struct_ptr = (uint16_t *)&App_data_array[3];
  FG_Reg_write(DesignCap, *struct_ptr);          // battery capacity in mAh.
  
  FG_Reg_write(IChgTerm,(uint16_t)(6.4 * App_data_array[6]));             // charge termination current in mA

  FG_Reg_write(Vempty, (uint16_t)App_data_array[6]);

  FG_Reg_write(Rsence, 0);

#if 0
  struct fuel_gauge Profile_data;
  Profile_data.FG.FullCapRep = Master_read(ModelCFG);
  Profile_data.FG.Cycles = Master_read(Rsence);
  Profile_data.FG.TimerH = Master_read(IChgTerm);
  Profile_data.FG.RepCap = Master_read(Vempty);
  Profile_data.FG.Rcell = Master_read(DesignCap);
  Profile_data.FG.Age = (Master_read(Rcell));
#endif

}
