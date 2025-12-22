{
  lib,
  stdenv,
  fetchurl,
  cctools,
  cmake,
  libtiff,
  libpng,
  zlib,
  git,
  libwebp,
  libraw,
  openexr,
  openjpeg,
  libjpeg,
  jxrlib,
  pkg-config,
  fixDarwinDylibNames,
}:

stdenv.mkDerivation (finalAttrs: {
  pname = "freeimage";
  version = "3.18.0-v0.3";

  src = fetchurl {
    # url = "https://github.com/agruzdev/FreeImageRe/archive/release";c
    url = "https://github.com/agruzdev/FreeImageRe/archive/refs/tags/latest.tar.gz";
    name = "latest.tar.gz";
    hash = "sha256-jZIn6Pn/O3CGQ0pLpXMv4OzoyzSuRYqJF2A4f44OjME=";
  };
  patches = [
      ./0001-fix-cmake-deps.patch
  ];

  nativeBuildInputs = [
    pkg-config
    cmake
    git
  ];

  buildInputs = [
    libtiff
    libraw
    libpng
    zlib
    libwebp
    openexr
    openjpeg
    libjpeg
    jxrlib
  ];


  INCDIR = "${placeholder "out"}/include";
  INSTALLDIR = "${placeholder "out"}/lib";
  cmakeFlags = [];

  preInstall = ''
    mkdir -p $INCDIR $INSTALLDIR
  '';


  # enableParallelBuilding = true;

  meta = {
    description = "FreeImage Re(surrected) - fork of the FreeImage library to maintain and extend";
    homepage = "https://github.com/agruzdev/FreeImageRe";
    license = "GPL";
  };
})
