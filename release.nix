{ nixpkgs ? import (builtins.fetchTarball https://github.com/tubleronchik/airapkgs/archive/nixos-unstable.tar.gz)
, system ? builtins.currentSystem
}:

let
  pkgs = nixpkgs { inherit system; };
in rec {
  package = pkgs.callPackage ./default.nix {  };
}
