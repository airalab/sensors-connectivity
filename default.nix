{ stdenv
, mkRosPackage
#, robonomics_comm-nightly
, ros_comm
, python3Packages
}:

mkRosPackage rec {
  name = "${pname}-${version}";
  pname = "sensors-connectivity";
  version = "0.3.0";

  src = ./.;

  propagatedBuildInputs = with python3Packages; [
    #robonomics_comm-nightly
    ros_comm
    setuptools
    pyserial
    requests
    sentry-sdk
    netifaces
    # protobuf
    pynacl
    ipfshttpclient
    paho-mqtt
    susbstrate-interface_0_13_6
  ];

  meta = with stdenv.lib; {
    description = "Agent that offers data from sensors";
    homepage = http://github.com/airalab/sensors-connectivity;
    license = licenses.bsd3;
    maintainers = with maintainers; [ vourhey ];
  };
}
