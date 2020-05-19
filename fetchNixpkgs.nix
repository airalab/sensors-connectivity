{ rev    ? "6ee06dbd030824d75ce933c2813a732ed2bb47ea"             # The Git revision of nixpkgs to fetch
, sha256 ? "12jql4dwa45kjnwfpp457sjmdrqv5i0bsrpd1lq2vhp45zwmqrdz" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
