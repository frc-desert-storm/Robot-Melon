{ pkgs ? import (fetchTarball "https://github.com/NixOS/nixpkgs/archive/refs/heads/nixos-unstable.tar.gz") {} }:

let
  gccLib = pkgs.gcc-lib or pkgs.stdenv.cc.lib or pkgs.gcc.cc.lib;
in
pkgs.mkShell {
  name = "robot-melon";

  nativeBuildInputs = with pkgs; [
    jdk17
    git
    libglvnd
    mesa
  ] ++ [ gccLib ];

  shellHook = ''
    export JAVA_HOME="${pkgs.jdk17.home}"
    export PATH="$JAVA_HOME/bin:$PATH"

    export LD_LIBRARY_PATH="${pkgs.lib.makeLibraryPath [gccLib]}"

    if [ -z "''${LC_ALL:-}" ] && [ -z "''${LANG:-}" ]; then
      export LC_ALL=C.UTF-8
    fi

    export LD_PRELOAD="${pkgs.libglvnd}/lib/libGLX.so.0:${pkgs.libglvnd}/lib/libGL.so.1''${LD_PRELOAD:+:$LD_PRELOAD}"
    if [ -d /run/opengl-driver/lib ]; then
      export __GLX_VENDOR_LIBRARY_DIR=/run/opengl-driver/lib
    else
      export __GLX_VENDOR_LIBRARY_DIR="${pkgs.mesa}/lib"
    fi

    cat <<'MSG'
    HIII
    MSG
  '';
}