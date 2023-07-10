BEGIN {
  indent="      "
  print indent, "# This list was generated using"
  print indent, "# doxygen -s -g Doxyfile.tmp"
  print indent, "# awk -f doxyfile.awk doxygen"
  print
}
/^#/ { print indent, $0 }
/^$/ { print indent, $0 }
/=/  { print indent, $1 }
