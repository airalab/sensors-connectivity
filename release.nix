{ nixpkgs ? import (builtins.fetchTarball https://github.com/tubleronchik/airapkgs/archive/fefa3fb823ca75fd4b269a40ac73dd587c62f9cb.tar.gz)
, system ? builtins.currentSystem
}:

let
  pkgs = nixpkgs { inherit system; };
in rec {
  package = pkgs.callPackage ./default.nix {  };
}
