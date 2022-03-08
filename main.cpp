#include "mbed.h"
#include "Adafruit_ADS1015.h"
 #include "MODSERIAL.h"
//#include "HighPWM.h"
#include "TextLCD.h"
#include "24LCxx_I2C.h"
#include "misra_types.h"
//
//---------------------------------------------------------------------------------------------
//
//13July2015
//1. mV drop to be divided by 4.53
//2. 1 output for Buzzer. 2 sec on after relay passes.
//3. Time out in case the unit does not detect relay has been removed.
//4. Potential free contact on for 5 sec after relay pass
//13July2015-end
//
// 21 AUG 2015: Buzzer re-assigned as PUNCH2 FOR MUX-Board.
//              DeltaV edit replaced with edit for mV drop
//
// 24 Aug 2015:Changes for OLD Hardware:
//              1. NO Indication( R, Y, G) when No relay is inserted.
//              2. ON Insertion YELLOW Indication till TEST is OVER/ABORTED/FAILED.
//              3. IF PASS Green indicator for 2 secs, PUNCH for 0.5 secs.
//              4. Start test again after 6 sec even if Relay is not removed.
//              5. Display PASS / FAIL count on Bottom Line.

//------------------------------------SUSHANT-------------------------------------------//
// 7 march 2016:Changes for OLD Hardware:To measure Relay Resistance.
//              1. we added a hardware module including 1)120E resistor,2)DPDT Relay,3)And ADS1115 ADC.
//------------------------------------SUSHANT-------------------------------------------//
//
//---------------------------------------------------------------------------------------------
//
#define MULTIPLEXER

#undef MULTIPLEXER


// to test whether a relay is plugged in: write  a low voltage to be output from DAC.
// OPA548 in turn will pass some current which is  compared by LM339 and further action taken.
#define TEST_VOLTAGE       (double)(1.50)
// voltage is measured with DMM when DAC has 0.5 written to it, this is enterd below
#define VOLT_OUT_HALFDAC   (double)(7.0)
#define DAC_TO_VOLTS       (double)( VOLT_OUT_HALFDAC / 0.500)
#define DAC_AT_12V         (double)(( 12.0 * 0.5)/ VOLT_OUT_HALFDAC)
#define DAC_AT_TEST        (double)(( TEST_VOLTAGE * 0.5)/ VOLT_OUT_HALFDAC)
#define I2CFREQ_EEP        ( 100000)

//----ADS1115----// 6/1/2016_SUSHANT

I2C i2c(P0_10, P0_11); //pin40-SDA , pin41-SCL
//I2C i2c(P0_19, P0_20); //pin71-SDA , pin72-SCL
//I2C i2c(P0_27, P0_28); //pin25-SDA , pin26-SCL





#define Coil_VOLTAGE       (float)(12.0)



#define EEPROM_START  ( 0x30)
#define ADR_PASS      ( EEPROM_START)
#define ADR_FAIL      ( ADR_PASS + sizeof(int))
#define ADR_CC1       ( ADR_FAIL + sizeof(double))
#define ADR_CC2       ( ADR_CC1 + sizeof(double))
#define ADR_PI_H      ( ADR_CC2 + sizeof(double))
#define ADR_PI_L      ( ADR_PI_H + sizeof(double))
#define ADR_DO_H      ( ADR_PI_L + sizeof(double))
#define ADR_DO_L      ( ADR_DO_H + sizeof(double))
#define ADR_SECURITY  ( ADR_DO_L + sizeof(double))

#ifdef MULTIPLEXER

    #define ADR_MVT       ( ADR_SECURITY + sizeof(double)) // millivolt drop test time
    #define ADR_MVDROP    ( ADR_MVT + sizeof(double))
    #define ADR_Rsense    ( ADR_MVDROP + sizeof(double))
    #define ADR_Rsense1   ( ADR_Rsense + sizeof(double))
    #define ADR_Rsense2   ( ADR_Rsense1 + sizeof(double))





#else


    #define ADR_Rsense1    ( ADR_SECURITY + sizeof(double))
    #define ADR_Rsense2    ( ADR_Rsense1 + sizeof(double))
    #define ADR_Cali_factor    ( ADR_Rsense2 + sizeof(double)) // calibration factor





#endif



//
//---------------------------------------------------------------------------------------------
//
// Host PC Communication channels
//Serial pc(USBTX, USBRX); // tx, rx
MODSERIAL    pc( USBTX, USBRX, 64, 64, "comport");
C24LCXX_I2C  eeprom( P0_19, P0_20, 0x00, NC, I2CFREQ_EEP);

//DigitalIn  key_start(P0_7);    //pin7

DigitalIn  key_enter(P0_0);     //pin9
DigitalIn  key_up   (P0_1);     //pin10
DigitalIn  key_right(P0_18);    //pin11
DigitalIn  key_dwn  (P0_17);    //pin12
DigitalIn  key_left (P0_15);    //pin13

DigitalIn  contact(P0_6);       //pin8
DigitalIn  binsens(P0_16);      //pin14
DigitalIn  plugin (P2_11);      //pin52
DigitalIn  coilshort(P2_12);    //pin53


DigitalOut EAled( P0_22);       //Led RED on EA1769;
DigitalOut RLY_R( P2_6);        //pin48
DigitalOut RLY_Y( P2_7);        //pin49
DigitalOut RLY_G( P2_8);        //pin50
DigitalOut Rly_dpdt( P0_24);       //pin16 used for measure Rsense_SUSHANT 19/2/16


DigitalOut PUNCH( P0_4);            //pin 38
#ifdef MULTIPLEXER

	DigitalOut MUX( P0_7);          //pin 7
    DigitalOut ENA( P0_9);          //pin 5
    DigitalOut PUNCH2( P0_8);       //pin 6
    DigitalOut RL8( P0_5);          //pin 39


    AnalogIn   Ain(P0_25);          //pin 17

  //  AnalogIn   Cin(P1_31);          //pin 20 relay_analog pin (23/12/2015) suryakant

    //AnalogIn   Bin(P0_24);        //pin 16    //05Aug2015
#endif

AnalogOut  dacout( P0_26);  //pin18

Ticker     blinky;
Ticker     to20ms;
Ticker     chatter;
Timer      timer;
Timer      mv_drop_Tmr;
Timeout    punch_tmout;
Timeout    pass_indicate_tmout;
Timeout    potfree_tmout;
Timeout    remove_tmout;
Timeout    disp_result_tmr;
Timer      RLY_Tmr;   //23/12/2015  suryakant

#define     PUNCH_TIME          0.5F
#define     RESULT_DISP_TIME    3.0F
#define     REMOVE_TIME         6.0F
#define     INDICATE_PASS_TIME  2.0F
#define     POT_FREE_TIME       3.0F
#define     MV_GAIN             3.35F


//TextLCD lcd(p15, p16, p17, p18, p19, p20, TextLCD::LCD16x4); // rs, e, d4-d7 ok
//TextLCD lcd(p15, p16, p17, p18, p19, p20, TextLCD::LCD20x2); // rs, e, d4-d7 ok
TextLCD lcd( P2_0, P2_1, P2_2, P2_3, P2_4, P2_5, TextLCD::LCD20x4); // rs, e, d4-d7 ok
//TextLCD lcd(p15, p16, p17, p18, p19, p20, TextLCD::LCD24x4); // rs, e, d4-d7 ok
//TextLCD lcd(p15, p16, p17, p18, p19, p20, TextLCD::LCD24x2); // rs, e, d4-d7 ok
//TextLCD lcd(p15, p16, p17, p18, p19, p20, TextLCD::LCD40x2); // rs, e, d4-d7 ok
//TextLCD_I2C lcd(&i2c_lcd, 0x42, TextLCD::LCD20x4); // I2C bus, PCF8574 Slaveaddress, LCD Type ok
//TextLCD_I2C lcd(&i2c_lcd, 0x42, TextLCD::LCD16x2, TextLCD::WS0010); // I2C bus, PCF8574 addr, LCD Type, Ctrl Type
//TextLCD_SPI lcd(&spi_lcd, p8, TextLCD::LCD24x4); // SPI bus, CS pin, LCD Type ok
//TextLCD_SPI lcd(&spi_lcd, p8, TextLCD::LCD40x2); // SPI bus, CS pin, LCD Type ok
//TextLCD_SPI lcd(&spi_lcd, p8, TextLCD::LCD40x4); // SPI bus, CS pin, LCD Type ok
//
//---------------------------------------------------------------------------------------------
//

static void init_vars( void);
static void read_vars_from_EEP( const short H_address, const short L_address, double *p_int_max, double *p_int_min);
static void read_vars_from_EEP1( const short address, double *p_int);
static bool chatter_test( int time_ms, int chTm);
static void chatter_flip( void);
static bool relay_internal_short( void);
static bool test_relay( void);
static void remove_relay( void);
static void print_results( void);
static void wait_for_bin_sense( void);
static void scan_kbd( void);
static void Housekeeping( void);
static void wait_for_key_release( uint8_t *key_name);
static void lcd_clr_line( int line_no);
static void lcd_draw_screen1( void);
static void lcd_draw_menu( void);
static void lcd_draw_counter( void);
static void lcd_draw_progview( void);         // mV drop
static void lcd_draw_progview1( void);        // coil resistance
static void lcd_draw_progview2( void);        // calibration factor
static uint8_t menu( void);
static uint8_t prog( uint8_t curr_State);
void rxCAR_RET( MODSERIAL_IRQ_INFO *q);
void cmd( void);
static void pass_isr( void);
static void disp_result_isr( void);

//30July2015
static void indicate( uint8_t status);
//30July2015-end


#ifdef MULTIPLEXER
    static void read_mV_drop( void);
    static void toggle_MUX( void);  //17June2015
    static void read_Rly_Vtg( void); //write function here (23/12/2015) suryakant
#endif

static void lcd_prn_counter( void);
//
//---------------------------------------------------------------------------------------------
//
#define RELEASED        (uint8_t)1
#define PRESSED         (uint8_t)0
#define CONTACT_CLOSED  (uint8_t)0
#define CONTACT_OPEN    (uint8_t)1

//30July2015
#define  NOT_INSERTED   (uint8_t)(0)
#define  TESTING        (uint8_t)(1)
#define  PASS           (uint8_t)(2)
#define  FAIL           (uint8_t)(3)
//30July2015-end

static uint8_t tower_status = NOT_INSERTED;
static int security = 0;

double pwm_val = 1.0;
//static double val = 0.0;
double F = 0.0;
double R = 0.0;

int cnt = 0;

static int cnt_pass = 0;
static int cnt_fail = 0;
static double set_pi_max = 8.00;
static double set_pi_min = 4.00;
static double set_do_max = 4.00;
static double set_do_min = 1.00;
static int chattertime1 = 1;
static int chattertime2 = 1;
static double  cali_factor = 1.00; // calibration factor








#ifdef MULTIPLEXER
    static float     mvdrop = 0.0f;
    static int   set_mvdrop = 300;
    static int   set_mvtime = 5;
    static float  Rsense    = 0.0f;
    static int i = 0;

    // coil variables for adc reading and timing
    //Suryakant

	static int coil_mvdrop = 50; //23/12/2015
	static int max_coil    = 2;
   	static int min_coil    = 1;





#else


    static float   Rsense = 0.0f; //23/12/2015
   	static int          i = 0;
    static int set_mvtime = 5;


   	static int max_coil    = 2;
   	static int min_coil    = 1;

   // static float 	cali_factor = 0.0; // calibration factor


#endif

static volatile bool Flag_rise = false;
static volatile bool Flag_fall = false;
static volatile bool Flag_Cmd_Rec = false;
static volatile bool Flag_20ms = false;
static volatile bool Flag_KeyEvent = false;
static volatile bool status_mux = false;
static volatile bool FLAG_RelayRemoved = false;
static volatile bool FLAG_displaying_prev_result = false;//05Aug2015

//30July2015
static volatile bool Flag_TestFailed = false;
//30July2015-end

static uint8_t Flag_key_left  = RELEASED;
static uint8_t Flag_key_dwn   = RELEASED;
static uint8_t Flag_key_enter = RELEASED;
static uint8_t Flag_key_up    = RELEASED;
static uint8_t Flag_key_right = RELEASED;
//static uint8_t Flag_key_start = RELEASED;


static bool chatter_over = true;

#define COARSE_INC  (0.001000)  // from 0.0 to 1.0
#define SCAN_PERIOD (1)         // time in secs
#define SEC_TO_USEC (1000000)
#define MECH_DELAY  (10)        // time in mSec

#define MAKE_DELAY   40
#define BREAK_DELAY  40

#define MENU        (uint8_t)(0)
#define TEST        (uint8_t)(1)
#define PROG        (uint8_t)(2)
#define VIEW        (uint8_t)(3)
#define COUNTER     (uint8_t)(4)

//
//---------------------------------------------------------------------------------------------
//





void r_trigger()
    {
    if( Flag_rise)
        {
        Flag_rise = false;
        EAled = 1;
        R = pwm_val; //dac_val;
        pc.printf("\r\nR:%f", pwm_val);
        }
    else{}
    }
//
//---------------------------------------------------------------------------------------------
//
void f_trigger()
    {
    if( Flag_fall)
        {
        Flag_fall = false;
        EAled = 0;
        F = pwm_val; //dac_val;
        pc.printf("\r\nF:%f", pwm_val);
        }
    else{}
    }
//
//---------------------------------------------------------------------------------------------
//
static void disp_result_isr( void)
    {
    FLAG_displaying_prev_result = false;
    disp_result_tmr.detach();
    }
//
//---------------------------------------------------------------------------------------------
//
static void punch_isr( void)
    {
    PUNCH = 0;
    #ifdef MULTIPLEXER
    PUNCH2 = 0;
    #endif
    punch_tmout.detach();
    }
//
//---------------------------------------------------------------------------------------------
//
static void pass_isr( void)
    {
    pass_indicate_tmout.detach();
    RLY_G = 0;
    }
//
//---------------------------------------------------------------------------------------------
//
static void remove_isr( void)
    {
    FLAG_RelayRemoved = true;
    remove_tmout.detach();
    }
//
//---------------------------------------------------------------------------------------------
//
static void blinky_isr( void)
    {
    static uint8_t hbstate = 0;

    blinky.detach();
    switch( hbstate)
        {
        case 0:
            EAled = 1;
            blinky.attach( &blinky_isr, 0.050);
            hbstate = 1;
        break;

        case 1:
            EAled = 0;
            blinky.attach( &blinky_isr, 0.150);
            hbstate = 2;
        break;

        case 2:
            EAled = 1;
            blinky.attach( &blinky_isr, 0.100);
            hbstate = 3;
        break;

        case 3:
        default:
            EAled = 0;
            blinky.attach( &blinky_isr, 0.700);
            hbstate = 0;
        break;
        }
    }
//
//------------------------------------------------------------------------
//

static void signature( void)
    {
    const char *author = "Copyright (c) 2015,2016 Pramod S. Joglekar";

    pc.rxBufferFlush();
    wait_ms(100);
    pc.printf("\r\nRelayTester %s %s", __DATE__, __TIME__);
    wait_ms(100);
    pc.printf("\r\n%s\r\n", author);
    }
//
//---------------------------------------------------------------------------------------------
//
void cmd( void)
    {
    char c[ 8];

    pc.move( c, 64 );

    if (( !strncmp( c, "PIN", sizeof("PIN")-1)) || ( !strncmp( c, "pin", sizeof("pin")-1)))
        {
        eeprom.Write( ADR_SECURITY, 0x5AA5);
        eeprom.Read( ADR_SECURITY, &security);
        pc.printf("\r\n%x:%x written", ADR_SECURITY, security);
        }
    else if( '?' == c[0])
        {
        signature();
        }
//     else if ((!strncmp( c, "uid", sizeof("uid")-1)) || (!strncmp( c, "UID", sizeof("UID")-1)))
//         {
//         unsigned char uid[33];

//         mbed_interface_uid( uid);
//
//         if( mbed_interface_uid( uid))
//             {
//             pc.printf("\r\nUnable to retrieve Serial Number from LPC\r\n");
//             }
//         else
//             {
//             pc.printf("\r\nSerial Number is: %s", uid);
//             }
//         }
    else if(( 'i' == c[0]) || ( 'I' == c[0]))
        {
        eeprom.Write( ADR_CC1,  1);
        eeprom.Write( ADR_CC2,  1);
        eeprom.Write( ADR_PI_H, (int)(8.0 * 100.0)); // convert 8.12 to 812
        eeprom.Write( ADR_PI_L, (int)(4.0 * 100.0));
        eeprom.Write( ADR_DO_H, (int)(4.0 * 100.0));
        eeprom.Write( ADR_DO_L, (int)(1.0 * 100.0));

        #ifdef MULTIPLEXER
            eeprom.Write( ADR_MVT, 5);
            eeprom.Write( ADR_Rsense1, (int)(max_coil * 10.0));
            eeprom.Write( ADR_Rsense2, (int)(min_coil * 10.0));
        #else

            eeprom.Write( ADR_Rsense1, (int)(max_coil * 10.0));
            eeprom.Write( ADR_Rsense2, (int)(min_coil * 10.0));
            eeprom.Write( ADR_Cali_factor, (int)(1.0 * 100.0)); // calibration factor
        #endif
        pc.printf("\r\nEEPROM initialised");
        }
//    else if(( 'R' == c[0]) || ( 'r' == c[0])) //"R0.7567<enter>"
//        {
//        coil = 1.0;
//
//        if( '+' == c[1])
//            {
//            val = val + 0.005;
//            pc.printf("\r\nval+ :%f", val);
//            }
//        else if( '-' == c[1])
//            {
//            val = val - 0.005;
//            pc.printf("\r\nval- :%f", val);
//            }
//        else
//            {
//            c[0] = ' ';
//            val = atof( c);
//            }
//
//        coil = val;
//        pc.printf("\r\ncoil :%f", val);
//        wait(0.5)                     ;
//        if( 0 == contact)
//            {
//            pc.printf("\r\nSUCCESS Contact CLOSED");
//            }
//        else
//            {
//            pc.printf("\r\nFAILED");
//            }
//        wait(2);
//        coil = 1.0;
//        }
//    else if(( 'F' == c[0]) || ( 'f' == c[0])) //"F0.7567<enter>"
//        {
//        c[0] = ' ';
//        val = atof( c);
//        pc.printf("\r\ncoil :%f", val);
//        coil = 0.0;
//        wait(0.5);
//        coil = val;
//        wait(0.5);
//        if( 1 == contact)
//            {
//            pc.printf("\r\nSUCCESS Contact OPEN");
//            }
//        else
//            {
//            pc.printf("\r\nFAILED");
//            }
//        wait(2);
//
//        coil = 1.0;
//        }
    else{}

    Flag_Cmd_Rec = false;
    for( int i = 0; i < 8; c[ i++] = '\0');    //{ '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'};
    }
//
//---------------------------------------------------------------------------------------------
//
static void timeout( void)
    {
    Flag_20ms = false;
    scan_kbd();
    }
//
//---------------------------------------------------------------------------------------------
//
static void init_vars( void)
    {
    eeprom.Read( ADR_PASS, &cnt_pass);
    if( 0 > cnt_pass)
        {
        cnt_pass = 0;
        eeprom.Write( ADR_PASS, cnt_pass);
        }
    else{}

    eeprom.Read( ADR_FAIL, &cnt_fail);
    if( 0 > cnt_fail)
        {
        cnt_fail = 0;
        eeprom.Write( ADR_FAIL, cnt_fail);
        }
    else{}

    eeprom.Read( ADR_CC1, &chattertime1);
    if( 5 < chattertime1)
        {
        chattertime1 = 1;
        eeprom.Write( ADR_CC1, chattertime1);
        }
    else if( 0 > chattertime1)
        {
        chattertime1 = 1;
        eeprom.Write( ADR_CC1, chattertime1);
        }
    else{}

    eeprom.Read( ADR_CC2, &chattertime2);
    if( 5 < chattertime2)
        {
        chattertime2 = 1;
        eeprom.Write( ADR_CC2, chattertime2);
        }
    else if( 0 >= chattertime2)
        {
        chattertime2 = 1;
        eeprom.Write( ADR_CC2, chattertime2);
        }
    else{}

#ifdef MULTIPLEXER
    eeprom.Read( ADR_MVT, &set_mvtime);
    if( 6 < set_mvtime)
        {
        set_mvtime = 5;
        eeprom.Write( ADR_MVT, set_mvtime);
        }
    else if( 0 >= set_mvtime)
        {
        set_mvtime = 5;
        eeprom.Write( ADR_MVT, set_mvtime);
        }
    else{}

    eeprom.Read( ADR_MVDROP, &set_mvdrop);
    if( 300 < set_mvdrop)
        {
        set_mvdrop = 300;
        eeprom.Write( ADR_MVDROP, set_mvdrop);
        }
    else if( 0 >= set_mvdrop)
        {
        set_mvdrop = 0;
        eeprom.Write( ADR_MVDROP, set_mvdrop);
        }
    else{}


    eeprom.Read( ADR_MVDROP, &set_mvdrop);
	if( 300 < set_mvdrop)
		{
		set_mvdrop = 300;
		eeprom.Write( ADR_MVDROP, set_mvdrop);
		}
	else if( 0 >= set_mvdrop)
		{
		set_mvdrop = 0;
		eeprom.Write( ADR_MVDROP, set_mvdrop);
		}
	else{}


	eeprom.Read( ADR_Rsense, &coil_mvdrop);
	if( 99 < coil_mvdrop)
		{
		coil_mvdrop = 99;
		eeprom.Write( ADR_Rsense, coil_mvdrop);
		}
	else if( 0 >= coil_mvdrop)
		{
		coil_mvdrop = 0;
		eeprom.Write( ADR_Rsense, coil_mvdrop);
		}
	else{}


	eeprom.Read( ADR_Rsense, &coil_mvdrop);
	if( 99 < coil_mvdrop)
		{
		coil_mvdrop = 99;
		eeprom.Write( ADR_Rsense, coil_mvdrop);
		}
	else if( 0 >= coil_mvdrop)
		{
		coil_mvdrop = 0;
		eeprom.Write( ADR_Rsense, coil_mvdrop);
		}
	else{}



//#else


	    eeprom.Read( ADR_Rsense1, &max_coil);
		if( 50 < max_coil)
			{
			max_coil = 50;

			eeprom.Write( ADR_Rsense1, max_coil);
			}
		else if( 0 >= max_coil)
			{
			max_coil = 0;
			eeprom.Write( ADR_Rsense1, max_coil);
			}
		else{}


		eeprom.Read( ADR_Rsense1, &max_coil);
		if( 50 < max_coil)
			{
			max_coil = 50;

			eeprom.Write( ADR_Rsense1, max_coil);
			}
		else if( 0 >= max_coil)
			{
			max_coil = 0;
			eeprom.Write( ADR_Rsense1, max_coil);
			}
		else{}

		eeprom.Read( ADR_Rsense2, &min_coil);
				if( 50 < min_coil)
					{
					min_coil = 50;
					eeprom.Write( ADR_Rsense2, min_coil);
					}
				else if( 0 >= min_coil)
					{
					min_coil = 0;
					eeprom.Write( ADR_Rsense2, min_coil);
					}
				else{}


				eeprom.Read( ADR_Rsense2, &min_coil);
				if( 50 < min_coil)
					{
					min_coil = 50;
					eeprom.Write( ADR_Rsense2, min_coil);
					}
				else if( 0 >= min_coil)
					{
					min_coil = 0;
					eeprom.Write( ADR_Rsense2, min_coil);
					}
				else{}




#else


			    eeprom.Read( ADR_Rsense1, &max_coil);
				if( 50 < max_coil)
					{
					max_coil = 50;

					eeprom.Write( ADR_Rsense1, max_coil);
					}
				else if( 0 >= max_coil)
					{
					max_coil = 0;
					eeprom.Write( ADR_Rsense1, max_coil);
					}
				else{}


				eeprom.Read( ADR_Rsense1, &max_coil);
				if( 50 < max_coil)
					{
					max_coil = 50;

					eeprom.Write( ADR_Rsense1, max_coil);
					}
				else if( 0 >= max_coil)
					{
					max_coil = 0;
					eeprom.Write( ADR_Rsense1, max_coil);
					}
				else{}

				eeprom.Read( ADR_Rsense2, &min_coil);
						if( 50 < min_coil)
							{
							min_coil = 50;
							eeprom.Write( ADR_Rsense2, min_coil);
							}
						else if( 0 >= min_coil)
							{
							min_coil = 0;
							eeprom.Write( ADR_Rsense2, min_coil);
							}
						else{}


						eeprom.Read( ADR_Rsense2, &min_coil);
						if( 50 < min_coil)
							{
							min_coil = 50;
							eeprom.Write( ADR_Rsense2, min_coil);
							}
						else if( 0 >= min_coil)
							{
							min_coil = 0;
							eeprom.Write( ADR_Rsense2, min_coil);
							}
						else{}



#endif

#ifdef MULTIPLEXER

    read_vars_from_EEP( ADR_PI_H, ADR_PI_L, &set_pi_max, &set_pi_min);
    read_vars_from_EEP( ADR_DO_H, ADR_DO_L, &set_do_max, &set_do_min);
#else
    read_vars_from_EEP( ADR_PI_H, ADR_PI_L, &set_pi_max, &set_pi_min);
    read_vars_from_EEP( ADR_DO_H, ADR_DO_L, &set_do_max, &set_do_min);
    read_vars_from_EEP1( ADR_Cali_factor, &cali_factor); // calibration factor
#endif

    }

//------------calibration factor--------------//
static void read_vars_from_EEP1( const short address, double *p_int)
    {
    int stored_max = 0;


    eeprom.Read( address, &stored_max);
    if( 1200 < stored_max)
        {
        stored_max = 100;
        eeprom.Write( address, stored_max);
        *p_int = 1.0;
        }
    else if( 1 > stored_max)
        {
        stored_max = 100;
        eeprom.Write( address, stored_max);
        *p_int = 1.0;
        }
    //-------------SUSHANT-------//
    else if( 0 >= stored_max)
           {
           stored_max = 100;
           eeprom.Write( address, stored_max);
           *p_int = 0.0;
           }
    //-------------SUSHANT-------//

    else
        {
        *p_int = ((double)(stored_max) / 100.0);
        }


    }

//------------calibration factor--------------//


//
//---------------------------------------------------------------------------------------------
//

static void read_vars_from_EEP( const short H_address, const short L_address, double *p_int_max, double *p_int_min)
    {
    int stored_max = 0;
    int stored_min = 0;

    eeprom.Read( H_address, &stored_max);
    if( 1200 < stored_max)
        {
        stored_max = 300;
        eeprom.Write( H_address, stored_max);
        *p_int_max = 3.0;
        }
    else if( 1 >= stored_max)
        {
        stored_max = 300;
        eeprom.Write( H_address, stored_max);
        *p_int_max = 3.0;
        }
    else
        {
        *p_int_max = ((double)(stored_max) / 100.0);
        }

    eeprom.Read( L_address, &stored_min);
    if( stored_max <= stored_min)
        {
        stored_min = ( stored_max > 1)? ( stored_max - 1) : 0;
        eeprom.Write( L_address, stored_min);
        *p_int_min = ((double)( stored_min) / 100.0);
        }
    else if( 0 >= stored_min)
        {
        stored_min = 0;
        eeprom.Write( L_address, stored_min);
        *p_int_min = 0.0;
        }
    else
        {
        *p_int_min = ((double)( stored_min) / 100.0);
        }
    }
//
//---------------------------------------------------------------------------------------------
//
#ifdef MULTIPLEXER  //17June2015
static void toggle_MUX( void)
    {
    status_mux = (true == status_mux)? false : true;
    MUX = status_mux;
    RL8 = status_mux;
    }
#endif //17June2015-end
//
//---------------------------------------------------------------------------------------------
//
static void indicate( uint8_t status)
    {
    switch( status)
        {
        case NOT_INSERTED:
            {
            RLY_R = 0;
            RLY_Y = 0;
            RLY_G = 0;
            }
        break;

        case TESTING:
            {
            RLY_R = 0;
            RLY_Y = 1;
            RLY_G = 0;
            }
        break;

        case PASS:
            {
            RLY_R = 0;
            RLY_Y = 0;
            RLY_G = 1;
            }
        break;

        case FAIL:
            {
            RLY_R = 1;
            RLY_Y = 0;
            RLY_G = 0;
            }
        break;

        default:
        break;
        }
    }
//
//---------------------------------------------------------------------------------------------
//
static void lcd_prn_counter( void)
    {
    lcd_clr_line( 3);
    lcd.locate( 0,3);
   // lcd.printf("PASS:%05d REJ:%5d", cnt_pass, cnt_fail);

    //23/12/25 suryakant
    lcd.printf("PASS:%05d ", cnt_pass ); //remove REJECT count string
    }
//
//---------------------------------------------------------------------------------------------
//
int main()
    {
    R = 0.0;
    F = 0.0;
    EAled = 0;
    tower_status = NOT_INSERTED;
    indicate( tower_status);
#ifdef MULTIPLEXER
    MUX = status_mux;
    RL8 = status_mux;
    ENA = 0;
#endif
    punch_isr();
    disp_result_isr();
    pass_isr();
    remove_isr();


    key_left.mode( PullUp);
    key_dwn.mode( PullUp);
    key_enter.mode( PullUp);
    key_up.mode( PullUp);
    key_right.mode( PullUp);
    contact.mode( PullUp);
    binsens.mode( PullUp);
    plugin.mode( PullUp);

    blinky.attach( &blinky_isr, 1);
    to20ms.attach_us( &timeout, 10000);
    dacout = DAC_AT_TEST;

    // change stdout to currently selected UART port
    char lineBuffer[256];

    freopen("/comport", "w", stdout);
    setvbuf(stdout, lineBuffer, _IOLBF, sizeof(lineBuffer));
    //fclose(stdout);
    //pc.baud(57600);
    pc.baud(9600); //( BAUD);    //(9600);
    pc.format(8,Serial::None,1);    /* format is 8 data bits, no stop bit, no parity */
    pc.txBufferFlush();
    pc.rxBufferFlush();
    pc.autoDetectChar('\r');  // only <ENTER> will be AutoDetected
    pc.attach(&rxCAR_RET, MODSERIAL::RxAutoDetect);
    pc.printf("\r\nRelay Testing");

    lcd.cls();
    lcd_draw_screen1();
    init_vars();

    eeprom.Read( ADR_SECURITY, &security);
  //  eeprom.Read( ADR_SECURITY, &security);
    //security = 0x5AA5;




    while(1)
        {


        static uint8_t State = TEST;

        Housekeeping();

        if( 0x5AA5 != security)
            {
            lcd.cls();
            lcd.setCursor(TextLCD::CurOff_BlkOff);
            lcd_clr_line( 0);
            lcd.locate( 0,0);
            lcd.printf(" SETUP ERROR        ");
            pc.printf("\r\nSETUP ERROR");
            wait(5);
            }
        else if( MENU == State)
            {
            State = menu();
            }
        else if( TEST == State) // default state
            {
            lcd.cls();
            lcd.setCursor(TextLCD::CurOff_BlkOff);
            lcd_draw_screen1();

            // stay stuck in here!!
            while( TEST == State)
                {

                //30July2015
                dacout = DAC_AT_TEST;
                wait(0.250);
                //30July2015-end

                Housekeeping();

                // First, test whether 2 keys are simultaneously pressed for 5 secs.
                if(( PRESSED == Flag_key_right) && ( PRESSED == Flag_key_dwn))
                    {
                    timer.reset();
                    timer.start();
                    while(( PRESSED == Flag_key_right) && ( PRESSED == Flag_key_dwn))
                        {
                        // scan keys every 0.1 sec, exit if release detected
                        wait(0.100);
                        if( 5 <= timer.read()) break;
                        }
                    timer.stop();

                    if( 5 <= timer.read())
                        {
                        // 2 keys pressed for 5 secs so MENU is selected
                        State = MENU; //exit condition
                        tower_status = NOT_INSERTED;
                        indicate( tower_status);
                        }
                    else
                        {} // do-nothing
                    }
                else if( 0 == plugin ) // else if relay is plugged in
                    {
                    dacout = DAC_AT_TEST;
                    wait(0.250);

                    // to make DPTP in NO condition

                    if( 0 == plugin )
                        {
                        if( test_relay())
                            {
                            wait_for_bin_sense();
                            }
                        else
#ifdef MULTIPLEXER
                            {
                            tower_status = NOT_INSERTED;
                            toggle_MUX();
                            }
#else
                            {
                            //till relay is unplugged OR timed-out
                           remove_relay();
                            }
#endif
                        }
                    else{} // false PLUGIN detection
                    }
                else
                    {
                    static bool Flag_Refresh = false;

                    wait_ms( 20);  // time-pass or scan-time

                    if( FLAG_displaying_prev_result)
                        {
                        // keep displaying previous result
                        Flag_Refresh = true;
                        }
                    else if( Flag_Refresh)
                        {
                        Flag_Refresh = false;
                        lcd_clr_line( 0);
                        lcd.locate( 0,0);
                        lcd.printf("    Relay Testing   ");
                        lcd_clr_line( 1);
                        lcd_clr_line( 2);
                        lcd.locate( 0,2);
                        lcd.printf("PLUGIN RELAY to TEST");
                        lcd_prn_counter();
                        remove_isr();
                        tower_status = NOT_INSERTED;
                        indicate( tower_status);
                        }
                    else{}
                    }
                }//end-while
            }
        else if( PROG == State)
            {
            tower_status = NOT_INSERTED;
            indicate( tower_status);
            State = prog( State);
            }
        else if( VIEW == State)
            {
            tower_status = NOT_INSERTED;
            indicate( tower_status);
            lcd.cls();
            lcd.setCursor( TextLCD::CurOff_BlkOff);
            lcd_draw_progview();

            while( VIEW == State)
                {
                if( PRESSED == Flag_key_enter)
                    {
                    wait_for_key_release( &Flag_key_enter);
                    State = MENU; //exit condition
                    }
                else if( PRESSED == Flag_key_right) // dummy code but required
                    {
                    wait_for_key_release( &Flag_key_right);
                    lcd_draw_progview();
                    }
                else{}
                }
            }
        else if( COUNTER == State)
            {
            tower_status = NOT_INSERTED;
            indicate( tower_status);
            lcd.cls();
            lcd.setCursor(TextLCD::CurOff_BlkOff);
            lcd_draw_counter();
            while( COUNTER == State)
                {
                if( PRESSED == Flag_key_enter)  //( PRESSED == Flag_key_right)
                    {
                    wait_for_key_release( &Flag_key_enter);
                    State = MENU; //exit condition
                    }
                else if( PRESSED == Flag_key_left)
                    {
                    wait_for_key_release( &Flag_key_left);
                    cnt_pass = 0;
                    cnt_fail = 0;
                    eeprom.Write( ADR_PASS, cnt_pass);
                    eeprom.Write( ADR_FAIL, cnt_fail);
                    lcd_draw_counter();
                    }
                else{}
                }
            }
        else{}
        }
    }
//
//---------------------------------------------------------------------------------------------
//
static uint8_t prog( uint8_t curr_State)
    {
    int col = 10;
    int row = 0;

    lcd.cls();
    lcd_draw_progview();
    lcd.locate( col, row);
    lcd.setCursor( TextLCD::CurOff_BlkOn);  //CurOn_BlkOff);    //

    while( PROG == curr_State)
        {
        if( PRESSED == Flag_key_enter)
            {
            wait_for_key_release( &Flag_key_enter);

            eeprom.Write( ADR_CC1,  chattertime1);
            eeprom.Write( ADR_CC2,  chattertime2);
            eeprom.Write( ADR_PI_H, (int)(set_pi_max * 100.0)); // convert 8.12 to 812
            eeprom.Write( ADR_PI_L, (int)(set_pi_min * 100.0));
            eeprom.Write( ADR_DO_H, (int)(set_do_max * 100.0));
            eeprom.Write( ADR_DO_L, (int)(set_do_min * 100.0));


            #ifdef MULTIPLEXER

            eeprom.Write( ADR_MVT,     set_mvtime);
            eeprom.Write( ADR_MVDROP,  (int)set_mvdrop);
            eeprom.Write( ADR_Rsense,  (int)coil_mvdrop);

            eeprom.Write( ADR_Rsense1, (int)(max_coil ));
            eeprom.Write( ADR_Rsense2, (int)(min_coil ));

            #else

            eeprom.Write( ADR_Rsense1, (int)(max_coil ));
            eeprom.Write( ADR_Rsense2, (int)(min_coil ));
            eeprom.Write( ADR_Cali_factor, (int)(cali_factor * 100.0 )); // calibration factor






            #endif

            curr_State = MENU; //exit condition   lcd.printf("C1:%1ds C2:%1ds CTm:%1dsec ", chattertime1, chattertime2, set_mvtime);
            }
        else if( PRESSED == Flag_key_right)
            {
            wait_for_key_release( &Flag_key_right);

            switch( row)
                {
                case 0:
                    {
                    if( 10 == col)  // @C1
                        {
                        col = 17;   // @C2
                        }
                    else
                        {
                        col = 5;    // PI H
                        row = 1;
                        }
                    }
                    break;

                case 1:
                case 2:
                    {
                    switch( col)
                        {
                        case 5:
                            col = 6;
                            break;

                        case 6:
                            col = 8;
                            break;

                        //case 7:  // decimal-point

                        case 8:
                            col = 9;
                            break;

                        case 9:
                            col = 14;
                            break;

                        case 14:
                            col = 15;
                            break;

                        case 15:
                            col = 17;
                            break;

                        //case 16:  // decimal-point

                        case 17:
                            col = 18;
                            break;

                        case 18:
                        default:
                            if( 2 == row)  //3
                                {
                                #ifdef MULTIPLEXER
                                      col = 4;

                                #else
                                      col = 10;
                                #endif
                                row = 3;    //0
                                }
                            else
                                {
                                col = 5;
                                ++row;
                                }
                            break;
                        }
                    }
                    break;

                case 3:
                    {
                    switch( col)
                        {
#ifdef MULTIPLEXER

  //------------------------------------SUSHANT-----------------------------------------//
                    //------mV Drop time and mV drop cursor setting.-----//

                   	     case 4:

							col = 8;
							break;

						case 8:
							col = 17;
							break;


						case 17:
							col = 18;
							break;


						case 18:
							col = 19;
							break;



						case 19:
							lcd_draw_progview1(); //Coil_Resistance program menu;


					       col = 0;
					       lcd.locate( 0, 3);

						    break;


			                 case 0:
							 col = 9;
							 break;

							 case 9:
							 col = 10;
							  break;

							 case 10:
							 col = 14;
							 break;

							 case 14:
							 col = 15;
							 break;

						     case 15:

						    	 col = 10;
						    	 row = 0;
						    	lcd_draw_progview();// mV drop program menu;




							break;

						default:
							col = 10;
						    row = 0;
							break;
		 //--------------------------SUSHAN-----------------------------------//



	#else

	 //--------------------------SUSHAN-----------------------------------//
			  //-------Coil resistance cursor setting.-------//

					 case 0:
							col = 9;
						 break;

						 case 9:
							col = 10;
						break;

						case 10:
							col = 14;
						break;

						case 14:
							col = 15;
					     break;
					     //----calibration factor---//
						case 15:
							lcd_draw_progview2(); // calibration factor program menu
							col = 16;
							break;
						case 16:
							col = 18;
							break;
						case 18:
							col = 19;
							break;


					     //----calibration factor---//


//--------------------------SUSHAN-----------------------------------//
						//case 15:
						case 19:

                        default:
                        	lcd_draw_progview();

                            if( 3 == row)
                                {
                                col = 10;
                                row = 0;
                                }
                            else
                                {
                                col = 5;
                                ++row;
                                }
                            break;
                        #endif
                        }
                    }
                    break;

                default:
                    col = 10;
                    row = 0;
                    break;
                }

            lcd.locate( col, row);
            lcd.setCursor( TextLCD::CurOff_BlkOn);  //CurOn_BlkOff);
            }
        else if( PRESSED == Flag_key_left)
            {
            wait_for_key_release( &Flag_key_left);

            switch( row)
                {
                case 0:
                    {
                    if( 17 == col)
                        {
                        col = 10;
                        }
                    else
                        {
                        #ifdef MULTIPLEXER
                        col = 19;
                        row = 3;
                        #else
                        col = 15;
                        row = 3;
                        #endif
                        }
                    }
                    break;

                case 1:
                case 2:
                    {
                    switch( col)
                        {
                        case 5:
                        default:
                            col = ( 1 == row)? 17:18;
                            --row;
                            break;

                        case 6:
                            col = 5;
                            break;

                        case 8:
                            col = 6;
                            break;

                        case 9:
                            col = 8;
                            break;

                        case 14:
                            col = 9;
                            break;

                        case 15:
                            col = 14;
                            break;

                        case 17:
                            col = 15;
                            break;

                        case 18:
                            col = 17;
                            break;
                        }
                    }
                    break;

                case 3:
                    {
                    switch( col)
                        {
                        #ifdef MULTIPLEXER
                        case 4:
                            col = 18;
                            row = 2;
                            break;
//-----------------------------------SUSHANT------------------------------//
//-----------mV Drop time and mV drop cursor setting----------------------//
                        case 19:
                        	col=18;
                            row=3;
                        break;


                        case 18:
                            col = 17;
                            row = 3;
                            break;

                        case 17:
                            col = 8;
                            row = 3;
                            break;

							case 8:

							col=4;
							row=3;
							break;
					default:

//--------------------------Coil_Resistance cursor setting-----------------------------------//
						if ( 3 == row)
							{
					switch(col)
						{
							case 15:
							col=14;
							row=3;
							break;

							case 14:
							col=10;
							row=3;
							break;

                            case 10:
                            col=9;
                            row=3;
                            break;

                            case 9:
                            default:
                            lcd_draw_progview(); // mV drop program menu
                            col=10;
                            row=0;

                            break;


									}
								}


                        #else
                        case 4:

                            col = ( 1 == row)? 17:18;
                            --row;
                            break;

                       //------calibration factor----//
                        case 19:
                        	col = 18;
                        	row = 3;
                        	break;
                        case 18:
                            col = 16;
                            row = 3;
                            break;
                        case 16:
                        	lcd_draw_progview1(); // coil resistance program menu
                            col = 15;
                            row = 3;
                            break;




                            //------calibration factor----//



                        case 15:
                        	col=14;
                        	row=3;
                        break;

                        case 14:
                        	col=10;
                        	row=3;
                        break;

                        case 10:
                             col=9;
                             row=3;
                        break;

                        case 9:
                           default:
                           lcd_draw_progview(); // mV drop program menu
                           col=4;
                           row=3;

                        #endif
                        }
                    }
                    break;

                default:
                    col = 10;
                    row = 0;
                    break;

  //--------------------------SUSHAN-----------------------------------//
                }
            lcd.locate( col, row);
            lcd.setCursor( TextLCD::CurOff_BlkOn);  //CurOn_BlkOff);
            }
        else if( PRESSED == Flag_key_up)
            {
            wait_for_key_release( &Flag_key_up);

            switch( row)
                {
                case 0:
                    {
                    switch( col)
                        {
                        case 10:
                            {
                            chattertime1 = ( 5 > chattertime1)? ( chattertime1 + 1) : chattertime1;
                            }
                            break;

                        case 17:
                            {
                            chattertime2 = ( 5 > chattertime2)? ( chattertime2 + 1) : chattertime2;
                            }
                            break;

                        default:
                            break;
                        }
                    }
                    break;

                case 1:
                    {
                    switch( col)
                        {
                        case 5:
                            {
                            set_pi_max = ( 2.0 > set_pi_max)? ( set_pi_max + 10.0) : set_pi_max;
                            }
                            break;

                        case 6:
                            {
                            set_pi_max = ( 11.0 > set_pi_max)? ( set_pi_max + 1.0) : set_pi_max;
                            }
                            break;

                        case 8:
                            {
                            set_pi_max = ( 11.9 > set_pi_max)? ( set_pi_max + 0.1) : set_pi_max;
                            }
                            break;
                        case 9:
                            {
                            set_pi_max = ( 12.0 > set_pi_max)? ( set_pi_max + 0.01) : set_pi_max;
                            }
                            break;

                        case 14:
                            {
                            set_pi_min = (( set_pi_max - 10.0) > set_pi_min)? ( set_pi_min + 10.0) : set_pi_min;
                            }
                            break;

                        case 15:
                            {
                            set_pi_min = (( set_pi_max - 1.0) > set_pi_min)? ( set_pi_min + 1.0) : set_pi_min;
                            }
                            break;

                        case 17:
                            {
                            set_pi_min = (( set_pi_max - 0.1) > set_pi_min)? ( set_pi_min + 0.1) : set_pi_min;
                            }
                            break;
                        case 18:
                            {
                            set_pi_min = (( set_pi_max - 0.01) > set_pi_min)? ( set_pi_min + 0.01) : set_pi_min;
                            }
                            break;

                        default:
                            break;
                        }
                    }
                    break;

                case 2:
                    {
                    switch( col)
                        {
                        case 5:
                            {
                            set_do_max = ( 2.0 > set_do_max)? ( set_do_max + 10.0) : set_do_max;
                            }
                            break;

                        case 6:
                            {
                            set_do_max = ( 11.0 > set_do_max)? ( set_do_max + 1.0) : set_do_max;
                            }
                            break;

                        case 8:
                            {
                            set_do_max = ( 11.9 > set_do_max)? ( set_do_max + 0.1) : set_do_max;
                            }
                            break;
                        case 9:
                            {
                            set_do_max = ( 12.0 > set_do_max)? ( set_do_max + 0.01) : set_do_max;
                            }
                            break;

                        case 14:
                            {
                            set_do_min = (( set_do_max - 10.0) > set_do_min)? ( set_do_min + 10.0) : set_do_min;
                            }
                            break;

                        case 15:
                            {
                            set_do_min = (( set_do_max - 1.0) > set_do_min)? ( set_do_min + 1.0) : set_do_min;
                            }
                            break;

                        case 17:
                            {
                            set_do_min = (( set_do_max - 0.1) > set_do_min)? ( set_do_min + 0.1) : set_do_min;
                            }
                            break;
                        case 18:
                            {
                            set_do_min = (( set_do_max - 0.01) > set_do_min)? ( set_do_min + 0.01) : set_do_min;
                            }
                            break;

                        default:
                            break;
                        }
                    }
                    break;

                case 3:
                    {
                    switch( col)
                        {
      #ifdef MULTIPLEXER
                            case 8:
                            {
                            set_mvtime = ( 6 > set_mvtime)? ++set_mvtime : set_mvtime;
                            }
                             break;

                            case 17:
                            {
                            set_mvdrop = (( 300 - 100) > set_mvdrop)? ( set_mvdrop + 100) : set_mvdrop;
                            }
                            break;

                            case 18:
                            {
                            set_mvdrop = (( 300 - 10) > set_mvdrop)? ( set_mvdrop + 10) : set_mvdrop;
                            }
                            break;

                            case 19:
                            {
                            set_mvdrop = (( 300 - 0) > set_mvdrop)? ( set_mvdrop + 1) : set_mvdrop;
                            }
                            break;
//#else



                            case 9:
							{
								max_coil = (( 50 - 10) > max_coil)? ( max_coil + 10) : max_coil;

							}
							break;


							case 10:
							{
								max_coil = (( 50 - 1) > max_coil)? ( max_coil + 1) : max_coil;
							}
							break;



							case 14:
							{
								min_coil = (( 50 - 10) > min_coil)? ( min_coil + 10) : min_coil;
							}
							break;


							case 15:
							{
								min_coil = (( 50 - 1) > min_coil)? ( min_coil + 1) : min_coil;
							}
							break;



    #else
							 case 9:
							{
						        max_coil = (( 50 - 10) > max_coil)? ( max_coil + 10) : max_coil;

							}
							 break;

							case 10:
							{
								max_coil = (( 50 - 1) > max_coil)? ( max_coil + 1) : max_coil;
							}
							 break;

							case 14:
							{
								min_coil = (( 50 - 10) > min_coil)? ( min_coil + 10) : min_coil;
							}
							break;

							case 15:
							{
								min_coil = (( 50 - 1) > min_coil)? ( min_coil + 1) : min_coil;
							}
							break;

							//-----------Calibration factor----------//
							  case 16:
							{
							    cali_factor = ( (1.00 - 1 ) >  cali_factor)? (  cali_factor + 1.00) :  cali_factor;
						    }
							break;
							 case 18:
	                        {
							    cali_factor = ((1.00 - 0.10) >  cali_factor)? (  cali_factor + 0.10) :  cali_factor;
							}
						    break;
							case 19:
							{
							   cali_factor = ((1.00 - 0.01) >  cali_factor)? (  cali_factor + 0.01) :  cali_factor;
							}
							break;

							//-----------Calibration factor----------//





                        default:
                            break;
                        #endif
                        }
                    }
                    break;

                default:
                    break;
                }
            {
    if(3 == row)
       {
 //--------------------------SUSHAN-----------------------------------//
            switch(col)
            {
#ifdef MULTIPLEXER
            case 8:
                lcd_draw_progview();
                break;

            case 17:
            	lcd_draw_progview();
            	break;

            case 18:
                 lcd_draw_progview();
                 break;
            case 19:
                 lcd_draw_progview();
                 break;

#else

//----------Calibration factor---------//
            case 18:
                  lcd_draw_progview2();
            break;

             case 19:
              lcd_draw_progview2();
            break;

#endif
             case 16:
              lcd_draw_progview2();
            break;
//----------Calibration factor---------//

            case 9:
                  lcd_draw_progview1();
                  break;
            case 10:
                  lcd_draw_progview1();
                  break;
            case 14:
                  lcd_draw_progview1();
                 break;
            case 15:
                  lcd_draw_progview1();
                  break;
            default:
                 break;

            }
       }

            else
            {
            	 lcd_draw_progview();
            }
       }

  //--------------------------SUSHAN-----------------------------------//

            lcd.locate( col, row);
            lcd.setCursor( TextLCD::CurOff_BlkOn);  //CurOn_BlkOff);
            }
        else if( PRESSED == Flag_key_dwn)
            {
            wait_for_key_release( &Flag_key_dwn);
            switch( row)
                {
                case 0:
                    {
                    switch( col)
                        {
                        case 10:
                            {
                            chattertime1 = ( 0 < chattertime1)? ( chattertime1 - 1) : chattertime1;
                            }
                            break;

                        case 17:
                            {
                            chattertime2 = ( 0 < chattertime2)? ( chattertime2 - 1) : chattertime2;
                            }
                            break;

                        default:
                            break;
                        }
                    }
                    break;

                case 1:
                    {
                    switch( col)
                        {
                        case 5:
                            {
                            set_pi_max = (( set_pi_min + 10.0) < set_pi_max)? ( set_pi_max - 10.0) : set_pi_max;
                            }
                            break;

                        case 6:
                            {
                            set_pi_max = (( set_pi_min + 1.0) < set_pi_max)? ( set_pi_max - 1.0) : set_pi_max;
                            }
                            break;

                        case 8:
                            {
                            set_pi_max = (( set_pi_min + 0.1) < set_pi_max)? ( set_pi_max - 0.1) : set_pi_max;
                            }
                            break;

                        case 9:
                            {
                            set_pi_max = (( set_pi_min + 0.01) < set_pi_max)? ( set_pi_max - 0.01) : set_pi_max;
                            }
                            break;

                        case 14:
                            {
                            set_pi_min = ( 10.0 < set_pi_min)? ( set_pi_min - 10.0) : set_pi_min;
                            }
                            break;

                        case 15:
                            {
                            set_pi_min = ( 1.0 < set_pi_min)? ( set_pi_min - 1.0) : set_pi_min;
                            }
                            break;

                        case 17:
                            {
                            set_pi_min = ( 0.1 < set_pi_min)? ( set_pi_min - 0.1) : set_pi_min;
                            }
                            break;

                        case 18:
                            {
                            set_pi_min = ( 0.01 < set_pi_min)? ( set_pi_min - 0.01) : set_pi_min;
                            }
                            break;

                        default:
                            break;
                        }
                    }
                    break;

                case 2:
                    {
                    switch( col)
                        {
                        case 5:
                            {
                            set_do_max = (( set_do_min + 10.0) < set_do_max)? ( set_do_max - 10.0) : set_do_max;
                            }
                            break;

                        case 6:
                            {
                            set_do_max = (( set_do_min + 1.0) < set_do_max)? ( set_do_max - 1.0) : set_do_max;
                            }
                            break;

                        case 8:
                            {
                            set_do_max = (( set_do_min + 0.1) < set_do_max)? ( set_do_max - 0.1) : set_do_max;
                            }
                            break;

                        case 9:
                            {
                            set_do_max = (( set_do_min + 0.01) < set_do_max)? ( set_do_max - 0.01) : set_do_max;
                            }
                            break;

                        case 14:
                            {
                            set_do_min = ( 10.0 < set_do_min)? ( set_do_min - 10.0) : set_do_min;
                            }
                            break;

                        case 15:
                            {
                            set_do_min = ( 1.0 < set_do_min)? ( set_do_min - 1.0) : set_do_min;
                            }
                            break;

                        case 17:
                            {
                            set_do_min = ( 0.1 < set_do_min)? ( set_do_min - 0.1) : set_do_min;
                            }
                            break;

                        case 18:
                            {
                            set_do_min = ( 0.01 < set_do_min)? ( set_do_min - 0.01) : set_do_min;
                            }
                            break;

                        default:
                            break;
                        }
                    }
                    break;

                case 3:
                    {
                    switch( col)
                        {
   #ifdef MULTIPLEXER  //coil_mvdrop
                    case 8:
                    {
                     set_mvtime = ( 1 < set_mvtime)? --set_mvtime : set_mvtime;
                    }
                    break;

                    case 17:
					{
					set_mvdrop = (( 0 + 100) < set_mvdrop)? ( set_mvdrop - 100) : set_mvdrop;
					}
					break;

					case 18:
					{
					set_mvdrop = (( 0 + 10) < set_mvdrop)? ( set_mvdrop - 10) : set_mvdrop;
					}
					break;

					case 19:
					{
					set_mvdrop = (( 0 + 0) < set_mvdrop)? ( set_mvdrop - 1) : set_mvdrop;
					}
					break;


//#else

					case 9:
					{
						max_coil = (( 0 + 10) < max_coil)? ( max_coil - 10) : max_coil;
					}
					break;


					case 10:
					{
						max_coil = (( 0 + 0) < max_coil)? ( max_coil - 1) : max_coil;
					}
					break;


					case 14:
					{
						min_coil = (( 0 + 10) < min_coil)? ( min_coil - 10) : min_coil;
					}
					break;

					case 15:
					{
						min_coil = (( 0 + 0) < min_coil)? ( min_coil - 1) : min_coil;
					}
					break;




      #else
					case 9:
		            {
						max_coil = (( 0 + 10) < max_coil)? ( max_coil - 10) : max_coil;
				    }
					break;


					case 10:
					{
						max_coil = (( 0 + 0) < max_coil)? ( max_coil - 1) : max_coil;
				    }
					break;



					case 14:
					{
					min_coil = (( 0 + 10) < min_coil)? ( min_coil - 10) : min_coil;
					}
					break;

					case 15:
					{
					min_coil = (( 0 + 0) < min_coil)? ( min_coil - 1) : min_coil;
					}
					break;
               //----------calibration factor----------//
				 case 16:
			    {
				cali_factor = ((0 + 1.0) < cali_factor)? ( cali_factor - 1.0) : cali_factor;
			    }
				break;
				 case 18:
			    {
				 cali_factor = (( 0 + 0.1) < cali_factor)? ( cali_factor - 0.1) : cali_factor;
			    }
				 break;

				case 19:
			    {
				 cali_factor = (( 0 + 0.01) < cali_factor)? ( cali_factor - 0.01) : cali_factor;
				}
			break;



			//----------calibration factor----------//




                        default:
                            break;
                        #endif
                        }
                    }
                    break;

                default:
                    break;
                }
            {

 //--------------------------SUSHAN-----------------------------------//

            if(3 == row)
            {
            switch(col)
                      {
            //------------calibration factor---------//
#ifdef MULTIPLEXER
                         case 8:
                        	lcd_draw_progview();
                        	break;

                      case 17:
                      	lcd_draw_progview();
                      	break;

                      case 18:
                           lcd_draw_progview();
                           break;
                      case 19:
                           lcd_draw_progview();
                           break;

 #else

                      case 18:
                           lcd_draw_progview2();
                         break;
                     case 19:
                         lcd_draw_progview2();
                       break;

#endif

                     case 16:
                         lcd_draw_progview2();
                        break;
         //------------calibration factor---------//


                      case 9:
                            lcd_draw_progview1();
                            break;
                      case 10:
                            lcd_draw_progview1();
                            break;
                      case 14:
                            lcd_draw_progview1();
                           break;
                      case 15:
                            lcd_draw_progview1();
                            break;
                      default:
                           break;
                      }
            }
            else
            {
            	 lcd_draw_progview();
            }
            }

//--------------------------SUSHAN-----------------------------------//

            lcd.locate( col, row);
            lcd.setCursor( TextLCD::CurOff_BlkOn);  //CurOn_BlkOff);
            }
        else{}
        }

    return curr_State;
    }
//
//---------------------------------------------------------------------------------------------
//
static uint8_t menu( void)
    {
    uint8_t row = 0;

    lcd.cls();
    lcd_draw_menu();
    // Show cursor as blinking character
    lcd.locate( 1, row);
    lcd.setCursor(TextLCD::CurOff_BlkOn);

    while(1)
        {
        if( PRESSED == Flag_key_dwn)
            {
            wait_for_key_release( &Flag_key_dwn);

            row = ++row % 4;
            // Show cursor as blinking character
            lcd.locate( 1, row);
            lcd.setCursor(TextLCD::CurOff_BlkOn);
            }
        if( PRESSED == Flag_key_up)
            {
            wait_for_key_release( &Flag_key_up);

            if( 0 == row)
                {
                row = 3;
                }
            else
                {
                row = --row;
                }
            // Show cursor as blinking character
            lcd.locate( 1, row);
            lcd.setCursor(TextLCD::CurOff_BlkOn);
            }
        else if( PRESSED == Flag_key_enter)  // enter
            {
            wait_for_key_release( &Flag_key_enter);

            return (row + 1);
            }
        else{}
        }
    }
//
//---------------------------------------------------------------------------------------------
//
static void lcd_draw_menu( void)
    {
    lcd.locate( 0,0);
    lcd.printf(" 1.  Relay Testing  ");
    lcd.locate( 0,1);
    lcd.printf(" 2.  Prog Mode      ");
    lcd.locate( 0,2);
    lcd.printf(" 3.  View Prog      ");
    lcd.locate( 0,3);
    lcd.printf(" 4.  Counter        ");
    }
//
//---------------------------------------------------------------------------------------------
//
static void lcd_draw_counter( void)
    {
    lcd.locate( 0,0);
    lcd.printf("       COUNTER      ");
    lcd.locate( 0,1);
    lcd.printf(" Passed: %d ", cnt_pass);
    lcd.locate( 0,2);
    lcd.printf(" Failed: %d ", cnt_fail);
    lcd.locate( 0,3);
    lcd.printf("RST:KEY-L  ESC:ENTER");
    }
//
//---------------------------------------------------------------------------------------------
//

static void lcd_draw_progview( void)
    {

    lcd.locate( 0,0);
    lcd.printf("CHTR C1: %2ds C2:%2ds ", chattertime1, chattertime2);
    //lcd.printf("C1:%1ds C2:%1ds CTm:%1ds ", chattertime1, chattertime2, set_mvtime);
    lcd.locate( 0,1);
    lcd.printf("PI H:%5.2fV L:%5.2fV ", set_pi_max, set_pi_min);
    lcd.locate( 0,2);
    lcd.printf("DO H:%5.2fV L:%5.2fV ", set_do_max, set_do_min);
    lcd.locate( 0,3);
   #ifdef MULTIPLEXER
    // previous lcd printf function
    //lcd.printf("CTm:%1d sec mVdrop:%03d", set_mvtime, set_mvdrop);

    	// updated 29/12/2015
    lcd.printf("mVdrp_t:%1ds    mV:%3d", set_mvtime,set_mvdrop);

    #else
    lcd.printf("Coil_R H:%2d L:%2d    " , max_coil, min_coil);

   // lcd.printf("Dt H:%5.2fV L:%5.2fV", set_delta_max, set_delta_min);

     //lcd.printf("DH:%2d L:%2d RH:%2dL:2d", set_delta_max, set_delta_min, set_delta_max_coil, set_delta_min_coil);
    #endif
    }

//---------SUSHANT--------//
static void lcd_draw_progview1( void)
    {


    lcd.locate( 0,0);
    lcd.printf("CHTR C1: %2ds C2:%2ds ", chattertime1, chattertime2);
    //lcd.printf("C1:%1ds C2:%1ds CTm:%1ds ", chattertime1, chattertime2, set_mvtime);
    lcd.locate( 0,1);
    lcd.printf("PI H:%5.2fV L:%5.2fV ", set_pi_max, set_pi_min);
    lcd.locate( 0,2);
    lcd.printf("DO H:%5.2fV L:%5.2fV ", set_do_max, set_do_min);
    lcd.locate( 0,3);
    #ifdef MULTIPLEXER

    // previous lcd printf function
    //lcd.printf("CTm:%1d sec mVdrop:%03d", set_mvtime, set_mvdrop);

    	// updated 29/12/2015
    //lcd.printf("mVdrp_t:%1ds    mV:%3d", set_mvtime,set_mvdrop);
    lcd.printf("Coil_R H:%2d L:%2d    " , max_coil, min_coil);
    #else
    lcd.printf("Coil_R H:%2d L:%2d    " , max_coil, min_coil);

   // lcd.printf("Dt H:%5.2fV L:%5.2fV", set_delta_max, set_delta_min);

    // lcd.printf("DH:%2d L:%2d RH:%2dL:2d", set_delta_max, set_delta_min, set_delta_max_coil, set_delta_min_coil);
    #endif
    }


//--------SUSHANT-----//

//--------------Calibration factor-----------//
static void lcd_draw_progview2( void)
    {


    lcd.locate( 0,0);
    lcd.printf("CHTR C1: %2ds C2:%2ds ", chattertime1, chattertime2);
    //lcd.printf("C1:%1ds C2:%1ds CTm:%1ds ", chattertime1, chattertime2, set_mvtime);
    lcd.locate( 0,1);
    lcd.printf("PI H:%5.2fV L:%5.2fV ", set_pi_max, set_pi_min);
    lcd.locate( 0,2);
    lcd.printf("DO H:%5.2fV L:%5.2fV ", set_do_max, set_do_min);
    lcd.locate( 0,3);
    #ifdef MULTIPLEXER

    // previous lcd printf function
    //lcd.printf("CTm:%1d sec mVdrop:%03d", set_mvtime, set_mvdrop);

    	// updated 29/12/2015
    //lcd.printf("mVdrp_t:%1ds    mV:%3d", set_mvtime,set_mvdrop);
    lcd.printf("Coil_R H:%2d L:%2d    " , max_coil, min_coil);
    #else
    lcd.printf("CALI FACTOR    :%1.2f " , cali_factor);

   // lcd.printf("Dt H:%5.2fV L:%5.2fV", set_delta_max, set_delta_min);

    // lcd.printf("DH:%2d L:%2d RH:%2dL:2d", set_delta_max, set_delta_min, set_delta_max_coil, set_delta_min_coil);
    #endif
    }



//--------------Calibration factor-----------//










//
//---------------------------------------------------------------------------------------------
//

static void lcd_clr_line( int line_no)
    {
    lcd.locate( 0, line_no);
    lcd.printf("                    ");
    }
//
//---------------------------------------------------------------------------------------------
//
static void lcd_draw_screen1( void)
    {
    remove_isr();
    tower_status = NOT_INSERTED;
    indicate( tower_status);

    lcd_clr_line( 0);
    lcd.locate( 0,0);
    lcd.printf("    Relay Testing   ");

    lcd_clr_line( 1);

    lcd_clr_line( 2);
    lcd.locate( 0,2);
    lcd.printf("PLUGIN RELAY to TEST");

    lcd_prn_counter();
    }
//
//---------------------------------------------------------------------------------------------
//
static void Housekeeping( void)
    {
    if( /*pc.txBufferEmpty() &&*/ ( Flag_Cmd_Rec))
        {
        cmd();
        pc.rxBufferFlush();
        }
    else
        {
        Flag_20ms = true;
        while( Flag_20ms)
            {;}
        //scan_kbd();
        }

    }
//
//---------------------------------------------------------------------------------------------
//
static void scan_kbd( void)
    {
    static uint8_t key_left_status = 0x00;
    static uint8_t key_right_status = 0x00;
    static uint8_t key_up_status = 0x00;
    static uint8_t key_dwn_status = 0x00;
    static uint8_t key_enter_status = 0x00;

    key_left_status = key_left_status << 1;
    key_left_status = ( 1 == key_left)? ( key_left_status | 0x01) : ( key_left_status & 0xFE);
    if(( RELEASED == Flag_key_left) && ( 0x00 == key_left_status))
        {
        Flag_key_left = PRESSED;
        Flag_KeyEvent = true;
        }
    else if(( PRESSED == Flag_key_left) && ( 0xff == key_left_status))
        {
        Flag_key_left = RELEASED;
        Flag_KeyEvent = true;
        }
    else{}

    key_right_status = key_right_status << 1;
    key_right_status = ( 1 == key_right)? ( key_right_status | 0x01) : ( key_right_status & 0xFE);
    if(( PRESSED == Flag_key_right) && ( 0xff == key_right_status))
        {
        Flag_key_right = RELEASED;
        Flag_KeyEvent = true;
        }
    else if(( RELEASED == Flag_key_right) && ( 0x00 == key_right_status))
        {
        Flag_key_right = PRESSED;
        Flag_KeyEvent = true;
        }
    else{}

    key_up_status = key_up_status << 1;
    key_up_status = ( 1 == key_up)? ( key_up_status | 0x01) : ( key_up_status & 0xFE);
    if(( PRESSED == Flag_key_up) && ( 0xff == key_up_status))
        {
        Flag_key_up = RELEASED;
        Flag_KeyEvent = true;
        }
    else if(( RELEASED == Flag_key_up) && ( 0x00 == key_up_status))
        {
        Flag_key_up = PRESSED;
        Flag_KeyEvent = true;
        }
    else{}

    key_dwn_status = key_dwn_status << 1;
    key_dwn_status = ( 1 == key_dwn)? ( key_dwn_status | 0x01) : ( key_dwn_status & 0xFE);
    if(( PRESSED == Flag_key_dwn) && ( 0xff == key_dwn_status))
        {
        Flag_key_dwn = RELEASED;
        Flag_KeyEvent = true;
        }
    else if(( RELEASED == Flag_key_dwn) && ( 0x00 == key_dwn_status))
        {
        Flag_key_dwn = PRESSED;
        Flag_KeyEvent = true;
        }
    else{}

    key_enter_status = key_enter_status << 1;
    key_enter_status = ( 1 == key_enter)? ( key_enter_status | 0x01) : ( key_enter_status & 0xFE);
    if(( PRESSED == Flag_key_enter) && ( 0xff == key_enter_status))
        {
        Flag_key_enter = RELEASED;
        Flag_KeyEvent = true;
        }
    else if(( RELEASED == Flag_key_enter) && ( 0x00 == key_enter_status))
        {
        Flag_key_enter = PRESSED;
        Flag_KeyEvent = true;
        }
    else{}
    }
//
//---------------------------------------------------------------------------------------------
//
static void wait_for_key_release( uint8_t *key_name)
    {
    while( PRESSED == *key_name)
        {
        Housekeeping();
        }
    }
//
//---------------------------------------------------------------------------------------------
//
void rxCAR_RET( MODSERIAL_IRQ_INFO *q)
    {
    Flag_Cmd_Rec = true;
    }
//
//---------------------------------------------------------------------------------------------
//
static void chatter_flip( void)
    {
    chatter_over = true;
    }
//
//---------------------------------------------------------------------------------------------
//
static bool chatter_test( int time_ms, int chTm)
    {
    chatter_over = false;
    chatter.attach( &chatter_flip, chTm);

    while( false == chatter_over)
        {
        timer.reset();
        timer.start();

        dacout = DAC_AT_12V; //dacout.write_u16( 0xffff);
        while( 1 == contact)
            {
            if( timer.read_ms() > MAKE_DELAY)
                {
                dacout.write_u16( 0x0000);
                return 1;
                }
            else{}
            }

        while( timer.read_ms() < time_ms)
            {}

        timer.reset();
        timer.start();
        dacout.write_u16( 0x0000);
        while( 0 == contact)
            {
            if( timer.read_ms() > BREAK_DELAY)
                {
                dacout.write_u16( 0x0000);
                return 1;
                }
            else{}
            }

        while( timer.read_ms() < time_ms)
            {}
        }
    chatter.detach();
    return 0;
    }
//
//---------------------------------------------------------------------------------------------
//
static bool relay_internal_short( void)
    {
    //---------------------------------------------------------------------------------
    // label JP4/1 as A, JP4/2 as B, JP2/1 as C and JP2/2 as D.
    // A - B SHORT: ( SHORT == true) && ( PLUGIN == true) when DAC = TEST VOLTAGE.
    // A - D SHORT: ( SHORT == true) && ( PLUGIN == false) when DAC = TEST VOLTAGE.
    // C - D SHORT / C - B SHORT:   ( CONTACT == false) when DAC = 0V.
    // A - C SHORT: Self chattering when DAC = 14V, PLUGIN toggles.
    // B - D SHORT: ( PLUGIN == false) even when RELAY is chattered.
    //---------------------------------------------------------------------------------


    // C - D SHORT / C - B SHORT:   ( CONTACT == false) when DAC = 0V.
    dacout.write_u16( 0x0000);
    wait_ms(100);
    if( CONTACT_CLOSED == contact)
        {
        lcd.locate( 0, 1);
        lcd.printf("  CONTACTS SHORTED  ");
        return true;
        }
    else
        {
        // A - B SHORT: ( SHORT == true) && ( PLUGIN == true) when DAC = TEST VOLTAGE.
        // A - D SHORT: ( SHORT == true) && ( PLUGIN == false) when DAC = TEST VOLTAGE.
        dacout = DAC_AT_TEST; //dacout.write_u16( 0xffff);
        wait_ms(50);
        if( true == coilshort)
            {
            lcd.locate( 0, 1);
            lcd.printf("  COIL SHORTED      ");
            return true;
            }
        else
            {
            dacout = DAC_AT_12V; //dacout.write_u16( 0xffff);
            wait_ms(50);  // for contact to close
            timer.reset();
            timer.start();
            while( timer.read_ms() < 200)
                {
                // A - C SHORT: Self chattering when DAC = 14V, PLUGIN toggles.
                if(( coilshort == true) && ( CONTACT_OPEN == contact))
                    {
                    lcd.locate( 0, 1);
                    lcd.printf("  COIL-CONTACT SHORT");
                    return true;
                    }
                else{}
                }
/*
            // B - D SHORT: ( PLUGIN == false) when RELAY is turned ON or chattered.
            if(( CONTACT_CLOSED == contact) && ( true == plugin))
                {
                return true;
                }
            else{}
*/
            }
        }

    return false;
    }
//
//---------------------------------------------------------------------------------------------
//
static bool test_relay( void)
    {
    static volatile bool Flag_VoltInc = true;
    static volatile bool Flag_FallingEdge = true;
    int inc_time = 0;
    int prev = 0;

    pc.printf("\r\nRelay Testing");

    tower_status = TESTING;

#ifdef MULTIPLEXER
    if( FLAG_displaying_prev_result)
        {
        // keep displaying previous result
        }
    else
#endif
        {
        indicate( tower_status);

        // clear previous result
        lcd_clr_line( 0);
        lcd.locate( 0,0);
        lcd.printf("    Relay Testing   ");

        lcd_clr_line( 1);//05Aug2015

        lcd_clr_line( 2);//05Aug2015
        }

    if( true == relay_internal_short())
        {
        #ifdef MULTIPLEXER
        // keep displaying previous result for 3secs
        while( FLAG_displaying_prev_result)
            {
            Housekeeping();
            }
        #endif

        tower_status = FAIL;
        indicate( tower_status);

        lcd_clr_line( 0);
        lcd.locate( 0,0);
        lcd.printf("    Test Result     ");

        lcd_clr_line( 1);
        lcd.locate( 0, 1);
        lcd.printf("   DEFECTIVE RELAY  ");

        return 1;
        }
    else{}

    if( 0 != chattertime1)
        {
        tower_status = TESTING;

        #ifdef MULTIPLEXER
        if( FLAG_displaying_prev_result)
            {
            // keep displaying previous result
            }
        else
        #endif
            {
            indicate( tower_status);

            // clear previous result//05Aug2015
            lcd_clr_line( 0);
            lcd.locate( 0,0);
            lcd.printf("    Relay Testing   ");

            lcd_clr_line( 1);//05Aug2015
            lcd.locate( 0, 1);
            lcd.printf("Chatter Cycle 1     ");

            lcd_clr_line( 2);//05Aug2015
            }
            wait_ms(100);

#ifdef MULTIPLEXER

            Rly_dpdt = 1;   //DPTP on

                        //---Resistance reading before starting of chattering cycle ----//

                                  // ads.setGain(GAIN_SIXTEEN); // ADS1115_set range to +/-0.256V
                                   // ads.setGain(GAIN_ONE);     // 1x gain   +/- 4.096V  1 bit = 2mV
                                   // ads.setGain(GAIN_TWO);     // 2x gain   +/- 2.048V  1 bit = 1mV
                                   // ads.setGain(GAIN_FOUR);    // 4x gain   +/- 1.024V  1 bit = 0.5mV
                                   // ads.setGain(GAIN_EIGHT);   // 8x gain   +/- 0.512V  1 bit = 0.25mV


                                   Adafruit_ADS1115 ads(&i2c);
                                   ads.setGain(  GAIN_ONE );     // 1x gain   +/- 4.096V  1 bit = 2mV
                                               wait_ms(100);
                                               Rsense = 0.0;
                                   uint16_t read_val = 0;


                                               for( i = 0; i < 64; i++)
                                               	{
                                               	read_val = ( read_val / 2) + ( ads.readADC_SingleEnded( 1) / 2) ;
                                              	wait_ms(2);
                                               	}

                                   // 0xffff    = 4.096V = 4096mV
                                   // read_val  = x
                                   // x = (( read_val * 4096) / 0xffff)mv;

                                   double temp = 0.0;
                                   temp = (double)( read_val * 4096);
                                   temp = ( temp / (double)0x7fff);

                                     temp = ( temp / 1000 );

                                   Rsense = temp;
                                   Rsense = (11.21 - Rsense );
                                   Rsense = (Rsense + 0.21);              //to match DMM value_SUSHANT
                                   Rsense = ( (temp * 120.5) / Rsense);
                                   Rsense = (Rsense - 1.12);

                           //--------calibration factor---------//
                                   if ( 0.01 == cali_factor)
                                   {

                                   }
                                   else
                                   {
                                   	Rsense = ( Rsense + cali_factor); // to match with the printed value
                                   }
                           //--------calibration factor---------//




                                   Rly_dpdt = 0;//DPTP off


#else

                                  Rly_dpdt = 1;   //DPTP on

             //---Resistance reading before starting of chattering cycle ----//

                       // ads.setGain(GAIN_SIXTEEN); // ADS1115_set range to +/-0.256V
                        // ads.setGain(GAIN_ONE);     // 1x gain   +/- 4.096V  1 bit = 2mV
                        // ads.setGain(GAIN_TWO);     // 2x gain   +/- 2.048V  1 bit = 1mV
                        // ads.setGain(GAIN_FOUR);    // 4x gain   +/- 1.024V  1 bit = 0.5mV
                        // ads.setGain(GAIN_EIGHT);   // 8x gain   +/- 0.512V  1 bit = 0.25mV


                        Adafruit_ADS1115 ads(&i2c);
                        ads.setGain(  GAIN_ONE );     // 1x gain   +/- 4.096V  1 bit = 2mV
                                    wait_ms(100);
                                    Rsense = 0.0;
                        uint16_t read_val = 0;


                                    for( i = 0; i < 64; i++)
                                    	{
                                    	read_val = ( read_val / 2) + ( ads.readADC_SingleEnded( 1) / 2) ;
                                   	wait_ms(2);
                                    	}

                        // 0xffff    = 4.096V = 4096mV
                        // read_val  = x
                        // x = (( read_val * 4096) / 0xffff)mv;

                        double temp = 0.0;
                        temp = (double)( read_val * 4096);
                        temp = ( temp / (double)0x7fff);

                          temp = ( temp / 1000 );

                        Rsense = temp;
                        Rsense = (11.21 - Rsense );
                        Rsense = (Rsense + 0.21);              //to match DMM value_SUSHANT
                        Rsense = ( (temp * 120.5) / Rsense);
                        Rsense = (Rsense - 1.12);

                //--------calibration factor---------//
                        if ( 0.01 == cali_factor)
                        {

                        }
                        else
                        {
                        	Rsense = ( Rsense + cali_factor); // to match with the printed value
                        }
                //--------calibration factor---------//




                        Rly_dpdt = 0;//DPTP off


#endif




        if( chatter_test( 10, chattertime1))
            {
          #ifdef MULTIPLEXER
            // wait and keep displaying previous result for 3secs
            while( FLAG_displaying_prev_result)
                {
                Housekeeping();
                }
            #endif
            tower_status = FAIL;
            indicate( tower_status);

            lcd_clr_line( 0);
            lcd.locate( 0,0);
            lcd.printf("    Test Result     ");

            lcd_clr_line( 1);
            lcd.locate( 0, 1);
            lcd.printf("ERR CT1: RLY CONTACT");

            return 1;
            }
        }
    else{}

    if( 0 != chattertime2)
        {
        tower_status = TESTING;

        #ifdef MULTIPLEXER
        if( FLAG_displaying_prev_result)
            {
            // keep displaying previous result
            }
        else
        #endif
            {
            indicate( tower_status);

            // clear previous result//05Aug2015
            lcd_clr_line( 0);
            lcd.locate( 0,0);
            lcd.printf("    Relay Testing   ");

            lcd_clr_line( 1);//05Aug2015
            lcd.locate( 0, 1);
            lcd.printf("Chatter Cycle 2     ");

            lcd_clr_line( 2);//05Aug2015
            }

        if( chatter_test( 100, chattertime2))
            {
            tower_status = FAIL;

            #ifdef MULTIPLEXER
            // wait and keep displaying previous result for 3secs
            while( FLAG_displaying_prev_result)
                {
                Housekeeping();
                }
            #endif

            indicate( tower_status);

            lcd_clr_line( 0);
            lcd.locate( 0,0);
            lcd.printf("    Test Result     ");

            lcd_clr_line( 1);
            lcd.locate( 0, 1);
            lcd.printf("ERR CT2: RLY CONTACT");

            return 1;
            }
        }
    else{}

    // Measure Time taken for TEST
    timer.reset();
    timer.start();

    tower_status = TESTING;

    #ifdef MULTIPLEXER
    if( FLAG_displaying_prev_result)
        {
        // keep displaying previous result
        }
    else
    #endif
        {
        indicate( tower_status);

        // clear previous result//05Aug2015
        lcd_clr_line( 0);
        lcd.locate( 0,0);
        lcd.printf("    Relay Testing   ");

        lcd_clr_line( 1);//05Aug2015
        lcd.locate( 0, 1);
        lcd.printf("Testing Relay - Ramp");

        lcd_clr_line( 2);//05Aug2015
        }

// find trigger points
    Flag_FallingEdge = true;
    Flag_VoltInc = true;
    Flag_fall = true;
    Flag_rise = true;
    prev = contact;
    // inc voltage from 0 to max in 1sec.
    // so time per increment = 1sec * 0.001 = 1msec.
    inc_time = (( SEC_TO_USEC * SCAN_PERIOD) * COARSE_INC);
    int dly = inc_time;

    for( cnt = 2; cnt > 0; )
        {
        if( false == Flag_VoltInc)
            {
            dacout = dacout + COARSE_INC;

            if( dacout > ( 1.0 - COARSE_INC))
                {
                Flag_VoltInc = true;
                }
            else{}
            }
       else
            {
            dacout = dacout - COARSE_INC;
            if( dacout < COARSE_INC)
                {
                Flag_VoltInc = false;   //true;

                Flag_fall = true;
                Flag_rise = true;
                cnt--;
                }
            else{}
            }
        pwm_val = dacout;

        wait_us( dly/3);

        if( prev != contact)
            {
            wait_ms(100);  // for contact bounce
            }

        if( Flag_FallingEdge && ( prev == 1) && ( 0 == contact))
            {
            Flag_FallingEdge = false;
            r_trigger();
            Flag_VoltInc = !Flag_VoltInc;
            // send voltage to 12V and wait 500 mSec
            dacout = DAC_AT_12V;
            wait_ms( 100);
            }
        else if( !Flag_FallingEdge && ( prev == 0) && ( 1 == contact))
            {
            Flag_FallingEdge = true;
            f_trigger();
            Flag_VoltInc = !Flag_VoltInc;
            Flag_fall = true;
            Flag_rise = true;
            cnt--;
            }
        else{}

        prev = contact;
        }

    F = F + 0.1;    // for rough DO //0.2
    R = R - 0.050;  // fot rough PI //0.1

    tower_status = TESTING;
    #ifdef MULTIPLEXER
    if( FLAG_displaying_prev_result)
        {
        // keep displaying previous result
        }
    else
    #endif
        {
        indicate( tower_status);

        // clear previous result//05Aug2015
        lcd_clr_line( 0);
        lcd.locate( 0,0);
        lcd.printf("    Relay Testing   ");

        lcd_clr_line( 1);//05Aug2015
        lcd.locate( 0, 1);
        lcd.printf("Testing Relay - PI  ");

        lcd_clr_line( 2);//05Aug2015
        }

// do slowly
    Flag_FallingEdge = true;
    Flag_VoltInc = false;   //true;
    Flag_fall = true;
    Flag_rise = true;
    prev = contact;
    dacout = R;
    wait(0.200); // L/R time-const

    for( cnt = 1; cnt > 0; )
        {
        if( false == Flag_VoltInc)
            {
            dacout = dacout + COARSE_INC;

            if( dacout > ( 1.0 - COARSE_INC))
                {
                pc.printf("\r\nPI TEST FAILED");
                Flag_VoltInc = true;
                Flag_TestFailed = true;

                tower_status = FAIL;

                #ifdef MULTIPLEXER
                // wait and keep displaying previous result for 3secs
                while( FLAG_displaying_prev_result)
                    {
                    Housekeeping();
                    }
                #endif

                indicate( tower_status);

                lcd_clr_line( 0);
                lcd.locate( 0,0);
                lcd.printf("    Test Result     ");

                lcd_clr_line( 1);
                lcd.locate( 0, 1);
                lcd.printf("PI TEST FAILED      ");

                lcd_clr_line( 2);//05Aug2015

                // create exit condition
                cnt--;
                }
            else
                {
                //30July2015
                Flag_TestFailed = false;
                //30July2015-end
                }
            }
       else
            {
            dacout = dacout - COARSE_INC;
            if( dacout < COARSE_INC)
                {
                pc.printf("\r\nTEST FAILED");
                Flag_VoltInc = false;
                Flag_TestFailed = true;

                tower_status = FAIL;

                #ifdef MULTIPLEXER
                // wait and keep displaying previous result for 3secs
                while( FLAG_displaying_prev_result)
                    {
                    Housekeeping();
                    }
                #endif

                indicate( tower_status);

                lcd_clr_line( 0);
                lcd.locate( 0,0);
                lcd.printf("    Test Result     ");

                lcd_clr_line( 1);
                lcd.locate( 0, 1);
                lcd.printf("TEST FAILED         ");

                // create exit condition
                cnt--;
                }
            else
                {
                Flag_TestFailed = false;
                }
            }

        if( false == Flag_TestFailed)
            {
            pwm_val = dacout;

            wait_us( inc_time * 25); // max of electrical-mechanical delay time (temd), and contact bounce .

            if( prev != contact)
                {
                wait_ms(100);  // for contact bounce
                }
            else{}

            if( Flag_FallingEdge && ( prev == 1) && ( 0 == contact))
                {
                Flag_FallingEdge = false;   // don't look for falling-edge
                r_trigger();
                Flag_VoltInc = true;

                // send voltage to 12V
                dacout = DAC_AT_12V;

                #ifdef MULTIPLEXER
                // send 100A current for 5 secs and read drop
                read_mV_drop();
                #else
                // and wait 500 mSec
                wait_ms( 500);

                #endif

                if( F > dacout)
                    {}
                else
                    {
                    dacout = F;
                    }

                tower_status = TESTING;

                #ifdef MULTIPLEXER
                if( FLAG_displaying_prev_result)
                    {
                    // keep displaying previous result
                    }
                else

                #endif
                    {
                    indicate( tower_status);
                    // clear previous result//05Aug2015
                    lcd_clr_line( 0);
                    lcd.locate( 0,0);
                    lcd.printf("    Relay Testing   ");

                    lcd_clr_line( 1);//05Aug2015
                    lcd.locate( 0, 1);
                    lcd.printf("Testing Relay - DO  ");

                    lcd_clr_line( 2);//05Aug2015
                    }
                }
            else if( !Flag_FallingEdge && ( prev == 0) && ( 1 == contact))
                {
                Flag_FallingEdge = true;
                f_trigger();
                Flag_VoltInc = false;

                if( 1 == contact)
                    {
                    pc.printf("\r\nSUCCESS Contact OPEN");
                    Flag_TestFailed = false;
                    }
                else
                    {
                    pc.printf("\r\nFAILED..Contact NOT OPEN");
                    Flag_TestFailed = true;
                    }

                // create exit condition
                cnt--;
                }
            else{}

            prev = contact;
            }
        else{}
        }

    dacout = 0.0;
    timer.stop();      // Measure Time taken for TEST
    print_results();

    return 0;
    }

//
//---------------------------------------------------------------------------------------------
//
#ifdef MULTIPLEXER
static void read_mV_drop( void)
    {

    ENA = 1;

    lcd.locate( 0, 1);
    lcd.printf("Testing - mV drop   ");
    wait_ms( 100);
    mvdrop = Ain.read();


    mv_drop_Tmr.reset();
    mv_drop_Tmr.start();
   while( mv_drop_Tmr.read() < set_mvtime) // read Analog I/P for 5 secs
        {
        wait_ms( 1);
        mvdrop = mvdrop + (( Ain.read() - mvdrop) / 25.0F);
        wait_ms(1);


       }



    ENA = 0;




    }
#endif


//
//---------------------------------------------------------------------------------------------
//
static void print_results( void)
    {
    double r_pi = ( R * DAC_TO_VOLTS);
    double r_do = ( F * DAC_TO_VOLTS);

    #ifdef MULTIPLEXER
    mvdrop = (( mvdrop * 3300.0f) / MV_GAIN);

    #endif

    if(set_pi_max < r_pi)
        {
        lcd_clr_line( 0);
        lcd.locate(0,0);
        lcd.printf("FAILED: PI HIGH     ");

        lcd_clr_line( 1);
        lcd.locate(0,1);
        lcd.printf("PI: %3.2fV  DO: %3.2fV  ", r_pi, r_do);

        wait_for_bin_sense();
        }
    else if(set_pi_min > r_pi)
        {
        lcd_clr_line( 0);
        lcd.locate(0,0);
        lcd.printf("FAILED: PI LOW      ");

        lcd_clr_line( 1);
        lcd.locate(0,1);
        lcd.printf("PI: %3.2fV  DO: %3.2fV  ", r_pi, r_do);

        wait_for_bin_sense();
        }
    else if(set_do_max < r_do)
        {
        lcd_clr_line( 0);
        lcd.locate(0,0);
        lcd.printf("FAILED: DO HIGH     ");

        lcd_clr_line( 1);
        lcd.locate(0,1);
        lcd.printf("PI: %3.2fV  DO: %3.2fV  ", r_pi, r_do);

        wait_for_bin_sense();
        }
    else if(set_do_min > r_do)
        {
        lcd_clr_line( 0);
        lcd.locate(0,0);
        lcd.printf("FAILED: DO LOW      ");

        lcd_clr_line( 1);
        lcd.locate(0,1);
        lcd.printf("PI: %3.2fV  DO: %3.2fV  ", r_pi, r_do);

        wait_for_bin_sense();
        }
#ifdef MULTIPLEXER
    else if( (float)set_mvdrop < mvdrop)
        {
        lcd_clr_line( 0);
        lcd.locate(0,0);
        lcd.printf("FAILED: mVolt HI    ");

        lcd_clr_line( 1);
        lcd.locate( 0, 1);
        lcd.printf("mV Drop: %3.0f mV ", mvdrop);
        wait_for_bin_sense();
        }

#else
     else if( (float)max_coil  < Rsense)
	   {
	   lcd_clr_line( 1);
	   lcd.locate(0,1);
	   lcd.printf("FAILED:Resi HI:%2.2f",Rsense);
	   wait_for_bin_sense();
	   }
       else if( (float)min_coil  > Rsense)
	   {
	   lcd_clr_line( 1);
	   lcd.locate(0,1);
	   lcd.printf("FAILED:Resi LW:%2.2f",Rsense);
	   wait_for_bin_sense();
	   }

    else if( Flag_TestFailed)
        {
        Flag_TestFailed = false;

        wait_for_bin_sense();
        }
#endif
    else
        {
        tower_status = PASS;
        indicate( tower_status);
        pass_indicate_tmout.attach( &pass_isr, INDICATE_PASS_TIME);

        lcd_clr_line( 0);
        lcd.locate( 0,0);
        lcd.printf("    Test Result     ");

        lcd_clr_line( 1);
        lcd.locate(0,1);
        lcd.printf("PI: %3.2fV  DO: %3.2fV  ", r_pi, r_do);

        #ifdef MULTIPLEXER
        lcd_clr_line( 2);
        lcd.locate( 0, 2);

        //lcd.printf("mV Drop: %3.0f mV ", mvdrop);


        //add coilvtg too 23/12/2015 suryakant
        lcd.printf("mV Drop:%3.0fmV R:%3.2f", mvdrop, Rsense);//%3.1f_sushant
        Rsense=0.00;
//---------------------------------------------------------------------------------------------------------
        if( status_mux)
            {
            PUNCH = 1;
            PUNCH2 = 0;
            }
        else
            {
            PUNCH = 0;
            PUNCH2 = 1;
            }

        FLAG_displaying_prev_result = true;
        disp_result_tmr.detach();
        disp_result_tmr.attach( &disp_result_isr, RESULT_DISP_TIME);
        Rsense = 0.00;
        #else
        lcd_clr_line( 2);
        lcd.locate( 0, 2);
        lcd.printf("Resi:%3.2f", Rsense);



        PUNCH = 1;
        FLAG_displaying_prev_result = false;
        disp_result_tmr.detach();
        #endif
        punch_tmout.attach( &punch_isr, PUNCH_TIME);
//---------------------------------------------------------------------------------------------------------

        ++cnt_pass;
        eeprom.Write( ADR_PASS, cnt_pass);
        lcd_prn_counter();

        }
    }
//
//---------------------------------------------------------------------------------------------
//
static void wait_for_bin_sense( void)
    {
    lcd_clr_line( 2);
    lcd.locate( 0,2);
    lcd.printf("    DROP IN BIN     ");


    Flag_TestFailed = true;
    tower_status = FAIL;
    indicate( tower_status);

//05Aug2015
    // wait until relay is un-plugged
    dacout = DAC_AT_TEST;
    wait_ms(50);  //(500);
    do
        {
        Housekeeping();
        wait_ms(10);
        }while( 0 == plugin );
//05Aug2015-end

    // wait for BINSENSE
    while( RELEASED == binsens)
        {
        Housekeeping();
        }

    ++cnt_fail;
    eeprom.Write( ADR_FAIL, cnt_fail);

    lcd_draw_screen1();
    }
//
//---------------------------------------------------------------------------------------------
//
static void remove_relay( void)
    {

    FLAG_RelayRemoved = false;
    remove_tmout.attach( &remove_isr, REMOVE_TIME);

    // exits after relay is un-plugged or time-out
    dacout = DAC_AT_TEST;
    wait_ms(50);  //(500);
    do
        {
        Housekeeping();
        wait_ms(10);
        }while(( 0 == plugin ) && ( false == FLAG_RelayRemoved));



    // Relay removed ie CONTACT OPEN ie ( 0 == plugin) is detected, or timed-out
    if( FLAG_RelayRemoved)
        {
        tower_status = TESTING;
        }
    else
        {
        tower_status = NOT_INSERTED;
        }

    indicate( tower_status);
    }
//
//---------------------------------------------------------------------------------------------
//
/*
static void remove_relay( void)
    {
    lcd.locate( 0, 3);
    lcd.printf("    REMOVE Relay    ");

    // excite relay to close contact, wait till it does, exit if it times out
    timer.reset();
    timer.start();
    dacout = DAC_AT_12V;
    wait_ms(50);

//    do
//        {
//        if( timer.read_ms() > MAKE_DELAY)
//            {
//            dacout.write_u16( 0x0000);
//            lcd.locate( 0, 3);
//            lcd.printf("ERROR: CONTACT OPEN ");
//
//            return 1; // TIME-OUT
//            }
//        else{}
//        wait_ms(10);
//        }while( 1 == contact);

    // comes here because "closed contact" is detected
    // wait till it opens, exit if it times out
    int debounce = 0;

    while( debounce < 10)   // stuck here till contact opens
        {
        bool Flg_TO = false;

        if( 1 == contact)
            {
            debounce++;
            }
        else if( 0 == contact)
            {
            debounce = 0;
            }
        else{}

        if(( timer.read() >= 60) && ( false == Flg_TO))
            {
            Flg_TO = true;
            timer.stop();
            lcd.locate( 0, 3);
            lcd.printf("ERROR:CONTACT STUCK ");
            }
        else{}

        Housekeeping(); //wait_ms(10);
        }

    // Relay removal ie CONTACT OPEN is detected
    lcd_draw_screen1();
    }
*/

//
//---------------------------------------------------------------------------------------------
//
/*
static void test_LCD( void)
    {
    pc.printf("LCD Test. Columns=%d, Rows=%d\n\r", lcd.columns(), lcd.rows());

    for (int row=0; row<lcd.rows(); row++)
        {
        int col=0;

        pc.printf("MemAddr(Col=%d, Row=%d)=0x%02X\n\r", col, row, lcd.getAddress(col, row));
//      lcd.putc('-');
        lcd.putc('0' + row);

        for (col=1; col<lcd.columns()-1; col++)
            {
            lcd.putc('*');
            }

        pc.printf("MemAddr(Col=%d, Row=%d)=0x%02X\n\r", col, row, lcd.getAddress(col, row));
        lcd.putc('+');
        }

// Show cursor as blinking character
    lcd.setCursor(TextLCD::CurOff_BlkOn);

// Set and show user defined characters. A maximum of 8 UDCs are supported by the HD44780.
// They are defined by a 5x7 bitpattern.
    lcd.setUDC(0, (char *) udc_0);  // Show |>
    lcd.putc(0);
    lcd.setUDC(1, (char *) udc_1);  // Show <|
    lcd.putc(1);
    lcd.setUDC(2, (char *) udc_2);
    lcd.putc(2);
    lcd.setUDC(3, (char *) udc_3);
    lcd.putc(3);
    lcd.setUDC(4, (char *) udc_4);
    lcd.putc(4);
    lcd.setUDC(5, (char *) udc_5);
    lcd.putc(5);
    lcd.setUDC(6, (char *) udc_6);
    lcd.putc(6);
    lcd.setUDC(7, (char *) udc_7);
    lcd.putc(7);
    }
*/
//
//---------------------------------------------------------------------------------------------
//
