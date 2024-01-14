#include <msp430.h>
#include <stdio.h>
#include <string.h>

// Define constants for various components
#define LED_OUTSIDE BIT5 // Define LED outside the home
#define light_sensor BIT4 // Light sensor to control the outside LED
#define servo BIT6 // Servo for opening or closing the door
#define LED_INSIDE BIT0 // LED inside the home, controlled by an app
#define DHT11 BIT4 // DHT11 temperature and humidity sensor
unsigned int ADC_Value; // Variable to store the analog value from the light sensor

// Variables for reading data from DHT11 sensor
volatile int temp[50];
volatile int diff[50];
volatile unsigned int i=0;
volatile unsigned int j=0;
char th_char[5];
char tl_char[5];
char hh_char[5];
char hl_char[5];
volatile int hh = 0;
volatile int hl = 0;
volatile int th = 0;
volatile int tl = 0;
volatile int check = 0;
volatile int checksum = 0;
volatile int dataok;

// Declaration of functions
void ser_output(char *str);
void itoa(int value, char *str, int base);
void itoa_float(float value, char *str);
void initUART(void);
void initADC(void);

// Main function
void main(void)
{
    // Microcontroller setup
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer
    BCSCTL1 = CALBC1_1MHZ; // Set DCO clock frequency
    DCOCTL = CALDCO_1MHZ;

    // GPIO setup for LEDs, light sensor, and servo
    P1DIR |= LED_OUTSIDE; // Set LED outside as output
    P1SEL |= light_sensor; // Select light sensor input

    // Servo setup
    P1DIR |= servo; // Set servo as output
    P1SEL |= servo; // Select servo timer

    // LED inside setup
    P1DIR |= LED_INSIDE; // Set inside LED as output

    // Delay for stabilization
    __delay_cycles(50000);

    // DHT11 sensor setup
    P2DIR |= DHT11; // đặt chân P2.4 là output để ...
    P2OUT &= ~DHT11;đặt mức thấp cho DHT11 để chuẩn bị cho quá trình nhận dữ liệu
    __delay_cycles(20000);// tạo độ dễ để chuẩn bị nhận dữ liệu từ DHT11
    P2OUT |= DHT11; // đặt mức cao cho DHT11 để bắt đầu nhận dữ liệu
    __delay_cycles(20);//tạo thời gian để DHT11 xử lí 
    P2DIR &= ~DHT11;// đặt chân P2.4 là input để nhận dữ liệu cụ thể là 40 bit
    P2SEL |= DHT11;

    // Initialize ADC and UART
    initADC();
    initUART();

    // Timer setup for DHT11
    TA1CTL = TASSEL_2 | MC_2; //nguồn đồng hồ là SMCLK (Sub-Main Clock), Continuous Mode,
    // nghĩa là timer sẽ hoạt động liên tục mà không cần phải cấu hình lại.
    TA1CCTL2 = CAP | CCIE | CCIS_0 | CM_2 | SCS; //là thanh ghi kiểm soát cho Capture/Compare channel 2 của Timer_A1.
 	//CAP chọn chế độ Capture Mode, nơi mà giá trị từ bộ đếm được lưu vào thanh ghi TA1CCR2.
	//CCIE kích hoạt ngắt cho Capture/Compare channel 2.
	//CCIS_0 chọn nguồn đầu vào cho chế độ Capture. Ở đây, CCIS_0 chọn chân đầu vào CCIxA (Capture/Compare Input) là chân P2.2.
	//CM_2 chọn chế độ Capture sự kiện trên lên và xuống.
	//SCS chọn nguồn đồng hồ cho Capture Mode. Ở đây, SCS đặt để chọn nguồn đồng hồ nội (SMCLK).
    // Enable global interrupts
    _BIS_SR(GIE);

    // Main loop
    while (1)
    {
        // Process data from DHT11, receive data with 32bit
        if (i >= 40)
        {	// bit đầu là để xác nhận ready to send
            for (j = 1; j <= 8; j++) // byte đầu là phần nguyên độ ẩm
            {
                if (diff[j] >= 110)
                    hh |= (0x01 << (8 - j));
            }
            for (j = 9; j <= 16; j++)// byte tiếp theo là phần thập phân độ ẩm
            {
                if (diff[j] >= 110)
                    hl |= (0x01 << (16 - j));
            }
            for (j = 17; j <= 24; j++) // byte tiếp theo là phần nguyên nhiệt độ
            {
                if (diff[j] >= 110)
                    th |= (0x01 << (24 - j));
            }
            for (j = 25; j <= 32; j++)// byte tiếp theo là phần thập phân nhiệt độ
            {
                if (diff[j] >= 110)
                    tl |= (0x01 << (32 - j));
            }
            for (j = 33; j <= 40; j++)// byte cuối là checksum
            {
                if (diff[j] >= 110)
                    checksum |= (0x01 << (40 - j));
            }
            check = hh + hl + th + tl;
            if (check == checksum)// kiểm tra xem data truyền và nhận đã đúng hay chưa
                dataok = 1;
            else
                dataok = 0;

            //Convert and send data over UART, chuyển đổi giá trị sang string để gửi đi
            char th_str[10], tl_str[5], hh_str[5], hl_str[5];
            char temperature[10], humidity[10];
            itoa(th, th_str, 10);
            itoa(tl, tl_str, 10);
            itoa(hh, hh_str, 10);
            itoa(hl, hl_str, 10);

            snprintf(temperature, sizeof(temperature), "%s.%s", th_str, tl_str);
            ser_output(temperature);
            ser_output(" ");

            snprintf(humidity, sizeof(humidity), "%s.%s", hh_str, hl_str);
            ser_output(humidity);
            ser_output(" ");

            //Read and process light sensor data
            ADC10CTL0 |= ENC + ADC10SC; // Enable and start conversion, Kích hoạt và bắt đầu chuyển đổi
            while (!(ADC10CTL0 & ADC10IFG)) {}; // Wait for conversion complete, Chờ quá trình chuyển đổi hoàn tất
            ADC_Value = ADC10MEM; // Read the result, Đọc kết quả

            char light_val[20];

            float adc_val = (float) ADC_Value;// đổi ADC_value sang float 

            adc_val = (1 - (adc_val / 1023.0)) * 100; //calculate brightness

            itoa_float(adc_val, light_val);// chuyển float sang string

            ser_output(light_val);

            // Control an LED based on ADC value
            if (ADC_Value > 0x2BC){ //dec  value > 700 turn on led
                P1OUT |= LED_OUTSIDE; // Set P1.5 high
            } else {
                P1OUT &= ~LED_OUTSIDE; // Set P1.5 low
            }

            __delay_cycles(1000000);
            //WDTCTL = WDT_MRST_0_064; // reset
        }
    }
}
// Timer interrupt service routine for DHT11
#pragma vector = TIMER1_A1_VECTOR
__interrupt void Timer_A1(void)
{
    temp[i] = TA1CCR2;TA1CCR2 là thanh ghi Capture/Compare Channel 2 của Timer_A1, chứa giá trị độ đo được trong chế độ Capture.
    i += 1;
    if (i >= 2)
        diff[i - 1] = temp[i - 1] - temp[i - 2];//tính toán sự thay đổi giữa các giá trị đo được từ lần đo trước và sau đó
    TA1CCTL2 &= ~CCIFG;
}
// UART interrupt service routine
#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void) {
    if (IFG2 & UCA0RXIFG) {
    char rx_val = UCA0RXBUF;
    TACCR0 = 20000;  //PWM period
           if(rx_val == '1')
           {
               P1OUT |= LED_INSIDE;
           }
           else if(rx_val == '0')
           {
               P1OUT &= ~LED_INSIDE;
           }
           else if(rx_val == '3')
           {
               TACCR1 = 350;  //CCR1 PWM Duty Cycle  !min 350 max 2600 angle 190,
               //350 2350-180 degrees
               TACCTL1 = OUTMOD_7;  //CCR1 selection reset-set
               TACTL = TASSEL_2|MC_1;   //SMCLK submain clock,upmode
               __delay_cycles(1500000);
           }
           else if(rx_val == '2')
           {
               TACCR1 = 1000;
               TACCTL1 = OUTMOD_7;  //CCR1 selection reset-set
               TACTL = TASSEL_2|MC_1;
               __delay_cycles(1500000);
           }
    }
}

//function send data over UART
void ser_output(char *str)
{
    while (*str != 0)
    {
       while (!(IFG2 & UCA0TXIFG));
        UCA0TXBUF = *str++;
    }
}
//int to char
void itoa(int value, char *str, int base)
{
    sprintf(str, "%d", value);
}
//float to char
void itoa_float(float value, char *str) {
    int int_part = (int)value;
        float decimal_part = value - (float)int_part;

        char int_str[10];
        itoa(int_part, int_str, 10);

        int decimal_int = (int)(decimal_part * 1000);

        char decimal_str[10];
        itoa(decimal_int, decimal_str, 10);

        strcpy(str, int_str);
        strcat(str, ".");
        strcat(str, decimal_str);
}

//setup UART
void initUART(void) {
    P1SEL |= BIT1 + BIT2;   // P1.1 = RXD, P1.2=TXD
    P1SEL2 |= BIT1 + BIT2;  // P1.1 = RXD, P1.2=TXD
    UCA0CTL1 |= UCSSEL_2;   // SMCLK
    UCA0BR0 = 104;          // 1MHz 9600
    UCA0BR1 = 0;            // 1MHz 9600
    UCA0MCTL = UCBRS_1;     // Modulation UCBRSx = 1
    UCA0CTL1 &= ~UCSWRST;   // Initialize USCI state machine
    IE2 |= UCA0RXIE;        // Enable USCI_A0 RX interrupt
}
//setup ADC
void initADC(void){
    ADC10AE0 = light_sensor; // Enable analog signal to A2
    ADC10CTL0 |= ADC10SHT_2; // 16 ADC10CLK cycles
    ADC10CTL0 |= ADC10ON; // Turn ADC10 on
    ADC10CTL1 |= ADC10SSEL_2; // ADC10 clock source = SMCLK
    ADC10CTL1 |= INCH_4; // ADC10 input channel = A2
}



