
#ifndef InputValues_h
#define InputValues_h

enum PositionSwitch
{
    Top,
    Middle,
    Bottom,
    Unknown
};

class InputValues
{
public:
    InputValues() {};
    double rightStickHorizontal;
    double rightStickVertical;
    double leftStickHorizontal;
    double leftStickVertical;
    double leftPoit;
    double rightPoti;

    PositionSwitch switchLeftOutside;
    PositionSwitch switchLeftInside;
    PositionSwitch switchRightOutside;
    PositionSwitch switchRightInside;

    bool failsafe;

private:
};

#endif