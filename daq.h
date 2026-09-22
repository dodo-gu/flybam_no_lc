#ifndef DAQ_H
#define DAQ_H

using namespace std;
using namespace FlyCapture2;
using namespace cv;

#define STEP_SIZE 1

class Daq
{
private:
	TaskHandle	taskHandleX;
	TaskHandle	taskHandleY;
	TaskHandle  taskHandleDig;

	float thetax, thetay;

	float64     dataX[STEP_SIZE];
	float64     dataY[STEP_SIZE];
	uInt8       dataDig[8];
	//uInt8		ifB0;
	//uInt8		ifB1;
	//uInt8		ifB2;

	TaskHandle taskHandleLens;
	float64 lensVoltage1; // Voltage for ao2
	float64 lensVoltage2; // Voltage for ao3

public:
	Daq();
	void reset();
	void configure();
	void start();
	void startTrigger();
	void stopTrigger();
	//void lensCommand(int cmd);
	void write();
	void flashHigh();
	void flashLow();
	//void ConvertPtToDeg(Point2f pt);
	void ConvertPixelToDeg(float x, float y);
	Point2f ConvertDegToPt();
	Point2f GetGalvoAngles();
	void SetGalvoAngles(Point2f angle);
	void MoveLeft();
	void MoveRight();
	void MoveUp();
	void MoveDown();
	//void PollLensPosition();
	void writeLens();
	// Lens 1 Fine Adjustment (ao2)
	void MoveFocus1Up();
	void MoveFocus1Down();
	// Lens 2 Fine Adjustment (ao3)
	void MoveFocus2Up();
	void MoveFocus2Down();

	void MoveBothFocusUp();
	void MoveBothFocusDown();
	// Presets
	void FocusMin();    // 0V
	void FocusMax();    // 10V
	void FocusDefault(); // default setting

	double getLensVoltage1() const { return lensVoltage1; }
	double getLensVoltage2() const { return lensVoltage2; }
	void setLensVoltage1(double v) { lensVoltage1 = v; }
	void setLensVoltage2(double v) { lensVoltage2 = v; }
};

#endif