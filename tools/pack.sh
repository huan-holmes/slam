version=$1
mkdir x86-release-${version}
cp -r tools x86-release-${version}
cp -r src_test x86-release-${version}
cp -r datasets x86-release-${version}
cp -r doc x86-release-${version}
cp -r install x86-release-${version}
cp README.md x86-release-${version}
zip x86-release-${version}.zip -r x86-release-${version}
rm -rf x86-release-${version}
