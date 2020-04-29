{ rev    ? "8beae7204d6c01d3da1645ecd8bf836cfc44446a"             # The Git revision of nixpkgs to fetch
, sha256 ? "1wdi5yrjxc0ipjfm2wp1hrs563rb6w2ipy07bbrh3686846ik5z2" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
