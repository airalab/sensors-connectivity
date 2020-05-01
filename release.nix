{ nixpkgs ? import ./fetchNixpkgs.nix { }
, system ? builtins.currentSystem
}:

let
  pkgs = import nixpkgs { inherit system; };

in rec {
  package = pkgs.callPackage ./default.nix { };
}
