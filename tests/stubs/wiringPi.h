// wiringPi.h, for the bench harnesses: modem_cw.c includes the real one
// and calls only millis() from it, so a PC without wiringPi can build
// tests/test_modem_cw.c, which provides millis().
unsigned int millis(void);
