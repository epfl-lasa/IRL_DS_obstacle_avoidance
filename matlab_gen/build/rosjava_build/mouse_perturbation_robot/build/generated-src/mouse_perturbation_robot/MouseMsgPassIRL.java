package mouse_perturbation_robot;

public interface MouseMsgPassIRL extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mouse_perturbation_robot/MouseMsgPassIRL";
  static final java.lang.String _DEFINITION = "# MouseMsgPassIRL.msg\n\n# float32[] xyz\n\ngeometry_msgs/Pose[] xyz";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  java.util.List<geometry_msgs.Pose> getXyz();
  void setXyz(java.util.List<geometry_msgs.Pose> value);
}
