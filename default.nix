{ stdenv
, mkRosPackage
, lib
#, robonomics_comm-nightly
, ros_comm
, python38Packages
, rospy
, catkin_38
}:

mkRosPackage rec {
  name = "${pname}-${version}";
  pname = "sensors-connectivity";
  version = "0.8.0";

  src = ./.;

  propagatedBuildInputs = with python38Packages; [
    #robonomics_comm-nightly
    ros_comm
    setuptools
    pyserial
    requests_38
    sentry-sdk
    netifaces
    # protobuf
    pynacl
    ipfshttpclient
    # substrate-interface_0_13_8
    paho-mqtt
    # xxhash
    pinatapy
    empy
    rospy
    catkin_38
    robonomics-interface
    # certifi38
  ];

  doCheck=false;
  meta = with lib; {
    description = "Agent that offers data from sensors";
    homepage = http://github.com/airalab/sensors-connectivity;
    license = licenses.bsd3;
    maintainers = with maintainers; [ vourhey ];
  };
}
