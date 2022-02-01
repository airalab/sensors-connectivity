{ nixpkgs ? import (builtins.fetchTarball https://github.com/airalab/airapkgs/archive/cb3f8021303f79345f65b5328b75117044bde852.tar.gz)
, system ? builtins.currentSystem
}:

let
  pkgs = nixpkgs { inherit system; };
in rec {
  package = pkgs.callPackage ./default.nix {  };
}
