let
  pkgs = import <nixpkgs> {};
  py-pkgs = p: with p; [
    ipython
    jupyter
    mypy

    matplotlib
    pygame
    numpy
    pandas
  ];
in
pkgs.mkShell {
  packages = [
    (pkgs.python39.withPackages py-pkgs)
  ];
}
