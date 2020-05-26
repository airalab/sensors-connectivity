{ stdenv
, fetchurl
, libgcc
, openssl
, zlib
, systemd
}:

let
  arch = if stdenv.hostPlatform.system == "x86_64-linux" then "x86_64"
    else if stdenv.hostPlatform.system == "aarch64-linux" then "linux-aarch64"
    else throw "Encryptr for ${stdenv.hostPlatform.system} not supported!";

  sha256 = if stdenv.hostPlatform.system == "x86_64-linux" then "1za3kf0lcw74z1sl6r5sixf7wyrw1ijchg30rc9fr41dk0pcpsrm"
    else if stdenv.hostPlatform.system == "aarch64-linux" then "0p2568fprrbfaxg0ci3hlxpd2kl0pz9d0h4yjrl0v524vixhxry4"
    else throw "Encryptr for ${stdenv.hostPlatform.system} not supported!";

in stdenv.mkDerivation rec {
   name = "substrate-node-robonomics-bin-${version}";
   version = "0.18.0";
   repoUrl = "https://github.com/airalab/robonomics";
   src = fetchurl {
      url = "${repoUrl}/releases/download/v${version}/robonomics-ubuntu-${version}-${arch}.tar.xz";
      inherit sha256;
   };
  dontBuild = true;
  sourceRoot = ".";
  libPath = stdenv.lib.makeLibraryPath
    [ libgcc
      openssl
      stdenv.cc.cc.lib # libstdc++.so.6
      zlib
      systemd.lib
    ];

  phases = "installPhase fixupPhase";

  installPhase = ''
   mkdir -p $out/bin
   tar -xf ${src} --directory $out/bin
   chmod +x $out/bin/robonomics
   patchelf --set-interpreter "$(cat $NIX_CC/nix-support/dynamic-linker)" \
   --set-rpath "$libPath" $out/bin/robonomics
  '';

   meta = {
      description = "Robonomics on Substrate node";
      homepage = https://github.com/airalab/substrate-node-robonomics;
      license = stdenv.lib.licenses.asl20;
      maintainers = [ "spd - spd@aira.life" ];
      platforms = [ "x86_64-linux" "aarch64-linux" ];
   };
 }

