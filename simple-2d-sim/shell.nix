let
  pkgs = import <nixpkgs> {};
  py-pkgs = p: with p; [
    ipython
    mypy

    pygame
    numpy
  ];
in
pkgs.mkShell {
  packages = [
    (pkgs.python39.withPackages py-pkgs)
  ];
}
