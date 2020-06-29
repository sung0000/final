#include "mbed.h"
#include "bbcar.h"

Serial pc(USBTX, USBRX);
DigitalOut redLed(LED1);
DigitalInOut pin10(D10);
parallax_ping ping(pin10);
Ticker encoder_ticker;
DigitalIn pin3(D3);
parallax_encoder encoder_left(pin3, encoder_ticker);
Ticker servo_ticker;
PwmOut pin8(D8), pin9(D9);
BBCar car(pin8, pin9, servo_ticker);
Serial uart(D1, D0);
RawSerial xbee(D12, D11);
EventQueue queue_xbee(32 * EVENTS_EVENT_SIZE);
Thread t_xbee;
Thread t_logger;

void logger();
void xbee_rx_interrupt(void);
void xbee_rx(void);
void reply_messange(char *xbee_reply, char *messange);
void check_addr(char *xbee_reply, char *messenger);

int count = 0;  // time counter
char state = '0';  // 0: don't move; S: straight; B: back; L: left; R: right


int main() {
    xbee.baud(9600);
    uart.baud(9600);

    redLed = 1;
	t_xbee.start(callback(&queue_xbee, &EventQueue::dispatch_forever));
    xbee.attach(xbee_rx_interrupt, Serial::RxIrq);
    t_logger.start(logger);

    // walk to mission 1
    wait(1);
    encoder_left.reset();
    state = 'S';
    car.goStraight(100);
    while (1){
        if ((float)ping < 30){
        car.stop();
        break;
        }
        wait_ms(10);
    }
    car.stop();
    
    wait(1);
    encoder_left.reset();
    state = 'L';
    car.turn(100, 0.3);
    while(encoder_left.get_cm()<21.5) wait_ms(50);
    car.stop();
    
    wait(1);
    encoder_left.reset();
    state = 'S';
    car.goStraight(100);
    while (1){
        if ((float)ping < 30){
        car.stop();
        break;
        }
        wait_ms(10);
    }
    car.stop();

    // reverse parking
    wait(1);
    encoder_left.reset();
    state = 'L';
    car.turn(-100, 0.3);
    while(encoder_left.get_cm()<26) wait_ms(50);
    car.stop();
    
    wait(1);
    encoder_left.reset();
    state = 'B';
    car.goStraight(-100);
    while (1){
        if ((float)ping > 45){
        car.stop();
        break;
        }
        wait_ms(10);
    }
    car.stop();

    // walk to mission 2
    wait(1);
    encoder_left.reset();
    state = 'R';
    car.turn(100, -0.3);
    while(encoder_left.get_cm()<14) wait_ms(50);
    car.stop();
    
    wait(1);
    encoder_left.reset();
    state = 'S';
    car.goStraight(100);
    while (1){
        if ((float)ping < 40){
        car.stop();
        break;
        }
        wait_ms(10);
    }
    car.stop();
    
    wait(1);
    encoder_left.reset();
    state = 'R';
    car.turn(100, -0.3);
    while(encoder_left.get_cm()<13.75) wait_ms(50);
    car.stop();
    
    wait(1);
    encoder_left.reset();
    state = 'S';
    car.goStraight(100);
    while (1){
        if ((float)ping < 40){
        car.stop();
        break;
        }
        wait_ms(10);
    }
    car.stop();
    
    wait(1);
    encoder_left.reset();
    state = 'R';
    car.turn(100, -0.3);
    while(encoder_left.get_cm()<13.75) wait_ms(50);
    car.stop();

    wait(1);
    encoder_left.reset();
    state = 'S';
    car.goStraight(100);
    while (1){
        if ((float)ping < 70){
        car.stop();
        break;
        }
        wait_ms(10);
    }
    car.stop();
    
    wait(1);
    encoder_left.reset();
    state = 'R';
    car.turn(100, -0.3);
    while(encoder_left.get_cm()<13.75) wait_ms(50);
    car.stop();
    
    wait(1);
    encoder_left.reset();
    state = 'S';
    car.goStraight(100);
    while (1){
        if ((float)ping < 15){
        car.stop();
        break;
        }
        wait_ms(10);
    }
    car.stop();

    // walk to the end
    wait(1);
    encoder_left.reset();
    state = 'B';
    car.goStraight(-100);
    while (1){
        if ((float)ping > 30){
        car.stop();
        break;
        }
        wait_ms(10);
    }
    car.stop();
    
    wait(1);
    encoder_left.reset();
    state = 'R';
    car.turn(-100, -0.3);
    while(encoder_left.get_cm()<10) wait_ms(50);
    car.stop();
    
    wait(1);
    encoder_left.reset();
    state = 'S';
    car.goStraight(100);
    while (1){
        if ((float)ping < 30){
        car.stop();
        break;
        }
        wait_ms(10);
    }
    car.stop();
    
    wait(1);
    encoder_left.reset();
    state = 'R';
    car.turn(100, -0.3);
    while(encoder_left.get_cm()<13.75) wait_ms(50);
    car.stop();
    
    wait(1);
    encoder_left.reset();
    state = 'S';
    car.goStraight(100);
    while(encoder_left.get_cm()<150) wait_ms(50);
    car.stop();
}

void logger(){
    while (1){
        if (non_mission){
            xbee.printf("%d %d\r\n", count, state);
            count++;
            wait(1);
        }else{
            wait(1);
        }
    }
}

void xbee_rx_interrupt(void)
{
	xbee.attach(NULL, Serial::RxIrq); // detach interrupt
	queue_xbee.call(&xbee_rx);
}

void xbee_rx(void)
{
	char buf[100] = {0};
	char outbuf[100] = {0};
	while (xbee.readable())
	{
		for (int i = 0;; i++)
		{
			char recv = xbee.getc();
			if (recv == '\r')
			{
				break;
			}
			buf[i] = pc.putc(recv);
		}
		pc.printf("%s\r\n", outbuf);
		wait(0.1);
	}
	xbee.attach(xbee_rx_interrupt, Serial::RxIrq); // reattach interrupt
}

void reply_messange(char *xbee_reply, char *messange)
{
	xbee_reply[0] = xbee.getc();
	xbee_reply[1] = xbee.getc();
	xbee_reply[2] = xbee.getc();
	if (xbee_reply[1] == 'O' && xbee_reply[2] == 'K')
	{
		pc.printf("%s\r\n", messange);
		xbee_reply[0] = '\0';
		xbee_reply[1] = '\0';
		xbee_reply[2] = '\0';
	}
}

void check_addr(char *xbee_reply, char *messenger)
{
	xbee_reply[0] = xbee.getc();
	xbee_reply[1] = xbee.getc();
	xbee_reply[2] = xbee.getc();
	xbee_reply[3] = xbee.getc();
	pc.printf("%s = %c%c%c\r\n", messenger, xbee_reply[1], xbee_reply[2], xbee_reply[3]);
	xbee_reply[0] = '\0';
	xbee_reply[1] = '\0';
	xbee_reply[2] = '\0';
	xbee_reply[3] = '\0';
}