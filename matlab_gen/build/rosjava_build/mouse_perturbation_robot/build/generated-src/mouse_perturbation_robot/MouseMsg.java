package mouse_perturbation_robot;

public interface MouseMsg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mouse_perturbation_robot/MouseMsg";
  static final java.lang.String _DEFINITION = "# MouseMsg.msg\n\nuint8 M_NONE = 0\nuint8 M_BTN_A = 1\nuint8 M_BTN_B = 2\nuint8 M_LEFT_CLICK = 3\nuint8 M_RIGHT_CLICK = 4\nuint8 M_WHEEL = 5\nuint8 M_CURSOR = 6\n\nuint8 event\nint32 buttonState\nint32 relX\nint32 relY\nint32 relZ\nint32 relWheel\nfloat32 filteredRelX\nfloat32 filteredRelY\nfloat32 filteredRelZ";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  static final byte M_NONE = 0;
  static final byte M_BTN_A = 1;
  static final byte M_BTN_B = 2;
  static final byte M_LEFT_CLICK = 3;
  static final byte M_RIGHT_CLICK = 4;
  static final byte M_WHEEL = 5;
  static final byte M_CURSOR = 6;
  byte getEvent();
  void setEvent(byte value);
  int getButtonState();
  void setButtonState(int value);
  int getRelX();
  void setRelX(int value);
  int getRelY();
  void setRelY(int value);
  int getRelZ();
  void setRelZ(int value);
  int getRelWheel();
  void setRelWheel(int value);
  float getFilteredRelX();
  void setFilteredRelX(float value);
  float getFilteredRelY();
  void setFilteredRelY(float value);
  float getFilteredRelZ();
  void setFilteredRelZ(float value);
}
