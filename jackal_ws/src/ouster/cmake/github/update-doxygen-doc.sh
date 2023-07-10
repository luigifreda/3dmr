#!/bin/bash
# Copyright (C) 2008-2017 LAAS-CNRS, JRL AIST-CNRS.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

 # ------ #
 # README #
 # ------ #

# This script updates the Doxygen documentation on Github.
#
# - it checkouts locally the gh-pages of the current repository,
# - it copy the local Doxygen documentation,
# - it creates a commit and push the modification.
#
# This scripts makes several assumptions on the project structure:
#
# 1. The documentation is generated in the doxygen-html directory
#    of the doc build directory.
#
# 2. The documentation is updated through `make doc` in the top build
#    directory.
#
# If doxygen.cmake is used, these assumptions will be respected.
#
#
# Launch the script by running:
#
# $ /path/to/update-doxygen-doc.sh -r /path/to/repo/root/dir \
#                                  -b /path/to/build/root/dir
set -e

# Override the locale.
LC_ALL='C'
export LC_ALL

me=$0
bme=`basename "$0"`


  # ----------------------- #
  # Customizable variables. #
  # ----------------------- #

: ${GIT=/usr/bin/git}

  # ---------------- #
  # Helper functions #
  # ---------------- #

# git_dependency_parsing
# ----------------------
#
# From an entry in GIT_DEPENDENCIES variable set git_dep, git_dep_uri and
# git_dep_branch in the environment
# For example given the input "jrl-umi3218/jrl-travis" the following variables
# are defined in the environment:
# - git_dep jrl-umi3218/jrl-travis
# - git_dep_uri git://github.com/jrl-umi3218/jrl-travis
# - git_dep_branch master
# - git_dep_github true
# - git_dep_organization jrl-umi3218
# - git_dep_project jrl-travis
# Or, given the input git@github.com:jrl-umi3218/jrl-travis#thebranch
# - git_dep jrl-umi3218/jrl-travis
# - git_dep_uri git@github.com:jrl-umi3218/jrl-travis
# - git_dep_branch thebranch
# - git_dep_github false
# The second (optional) argument allows to defined the default branch (defaults
# to master)
git_dependency_parsing()
{
  _input=$1
  export git_dep=${_input%%#*}
  export git_dep_branch=${_input##*#}
  if [ "$git_dep_branch" == "$git_dep" ]; then
    if [ -e "$2" ]; then
      export git_dep_branch=$2
    else
      export git_dep_branch="master"
    fi
  fi
  git_dep_uri_base=${git_dep%%:*}
  if [ "$git_dep_uri_base" == "$git_dep" ]; then
    export git_dep_uri="git://github.com/$git_dep"
    export git_dep_github=true
    export git_dep_organization=${git_dep%%/*}
    export git_dep_project=${git_dep##*/}
  else
    export git_dep_uri=$git_dep
    export git_dep=${git_dep##*:}
    export git_dep_github=false
  fi
}

set_colors()
{
  red='[0;31m';    lred='[1;31m'
  green='[0;32m';  lgreen='[1;32m'
  yellow='[0;33m'; lyellow='[1;33m'
  blue='[0;34m';   lblue='[1;34m'
  purple='[0;35m'; lpurple='[1;35m'
  cyan='[0;36m';   lcyan='[1;36m'
  grey='[0;37m';   lgrey='[1;37m'
  white='[0;38m';  lwhite='[1;38m'
  std='[m'
}

set_nocolors()
{
  red=;    lred=
  green=;  lgreen=
  yellow=; lyellow=
  blue=;   lblue=
  purple=; lpurple=
  cyan=;   lcyan=
  grey=;   lgrey=
  white=;  lwhite=
  std=
}

# abort err-msg
abort()
{
  echo "update-doxygen-doc.sh: ${lred}abort${std}: $@" \
  | sed '1!s/^[ 	]*/             /' >&2
  exit 1
}

# warn msg
warn()
{
  echo "update-doxygen-doc.sh: ${lred}warning${std}: $@" \
  | sed '1!s/^[ 	]*/             /' >&2
}

# notice msg
notice()
{
  echo "update-doxygen-doc.sh: ${lyellow}notice${std}: $@" \
  | sed '1!s/^[ 	]*/              /' >&2
}

# yesno question
yesno()
{
  printf "$@ [y/N] "
  read answer || return 1
  case $answer in
    y* | Y*) return 0;;
    *)       return 1;;
  esac
  return 42 # should never happen...
}


  # -------------------- #
  # Actions definitions. #
  # -------------------- #

version()
{
    echo 'update-doxygen-doc.sh
Copyright (C) 2008-2017 LAAS-CNRS, JRL CNRS/AIST.
This is free software; see the source for copying conditions.
There is NO warranty; not even for MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.'
}

help()
{
    echo 'Usage: update-doxygen-doc.sh -r ROOT_DIR -b BUILD_DIR [options]

-r and -b are mandatory. They must be set respectively to the project
root directory and to the build directory.

Options:
  --help, -h		      Print this message and exit.
  --version, -v		      Print the script version and exit.
  --doc-version VERSION       Update the documentation of a stable
                              release to github.
                              VERSION by default is HEAD but can be
                              changed to vX.Y.Z to update the documentation
                              of a released version.

Report bugs to http://github.com/jrl-umi3218/jrl-cmakemodules/issues
For more information, see http://github.com/jrl-umi3218/jrl-cmakemodules
'

}
  # ------------------- #
  # `main' starts here. #
  # ------------------- #

# Define colors if stdout is a tty.
if test -t 1; then
  set_colors
else # stdout isn't a tty => don't print colors.
  set_nocolors
fi

# For dev's:
test "x$1" = x--debug && shift && set -x

# Default documentation version
doc_version=HEAD
# Remote URL used to push
remote_url=
# Package root directory
root_dir=
# Build root directory
build_dir=

while `test $# -gt 0`; do
    case $1 in
	version | -v | --version | -version)
	    shift
	    version
	    exit 0
	    ;;
	help | -h | --help | -help)
	    shift
	    help
	    exit 0
	    ;;
	--doc-version)
	    doc_version=$2
	    shift; shift
	    ;;
	--remote-url)
	    remote_url=$2
	    shift; shift
	    ;;
	--root-dir | -r)
	    root_dir=$2
	    shift; shift
	    ;;
	--build-dir | -b)
	    build_dir=$2
	    shift; shift
	    ;;
	*)
	    echo "update-doxygen-doc.sh: ${lred}invalid option${std}: $1"
	    shift
	    help
	    exit 1
	    ;;
    esac
done

# If root dir is not set, fail.
if `test x${root_dir} = x`; then
    abort "root directory is not set, please set it using \`-r'"
    usage
    exit 1
fi
# If build dir is not set, fail.
if `test x${build_dir} = x`; then
    abort "build directory is not set, please set it using \`-b'"
    usage
    exit 1
fi

# If no remote URL is set, try to retrieve it automatically.
if `test x${remote_url} = x`; then
    cd $root_dir
    remote_url=`${GIT} config remote.origin.url`
fi


# Main starts here...
echo "* Checkout ${doc_version}..."
cd $root_dir
${GIT} checkout --quiet $doc_version

echo "* Generating the documentation..."
cd $build_dir
make doc 2> /dev/null > /dev/null || \
 abort "failed to generate the documentation"

echo "* Creating the temporary directory..."
tmp=`mktemp -d` || abort "cannot create the temporary directory"
trap "rm -rf -- '$tmp'" EXIT

cd $root_dir
head_commit=`${GIT} rev-parse HEAD`


echo "* Clone the project..."
cd $tmp
${GIT} clone --quiet --depth 1 --branch gh-pages $remote_url project \
 || abort "failed to clone the package repository"
cd ${tmp}/project \
 || abort "failed to change directory"

echo "* Copy the documentation..."
git rm --quiet -rf doxygen/${doc_version} || true
mkdir -p doxygen/${doc_version}
cp -rf ${build_dir}/doc/doxygen-html/* ${build_dir}/doc/*.doxytag doxygen/${doc_version}/ \
 || abort "failed to copy the documentation"

echo "* Fixing cross links..."
cd "$tmp/project/doxygen/$doc_version"
for package in ${GIT_DEPENDENCIES}
do
  git_dependency_parsing $package
  if [ $git_dep_github = true ]; then
    doxygen_url="https://${git_dep_organization}.github.io/${git_dep_project}/doxygen/HEAD/"
    git_dep_pc=`find /tmp/_ci/build/$git_dep \( ! -regex '.*/\..*' \) -name '*.pc'`
    git_dep_pc=`basename $git_dep_pc .pc`
    git_dep_doxygen=`pkg-config --variable=doxygendocdir $git_dep_pc || echo ""`
    if `test x${git_dep_doxygen} != x`; then
      find . -type f -print0 |xargs -0 sed -i "s@$git_dep_doxygen@$doxygen_url@g"
    fi
  fi
done
cd "$tmp/project/"

echo "* Generate the commit..."
${GIT} add doxygen/$doc_version \
 || abort "failed to add the updated documentation to the git index"

echo "Update $doc_version Doxygen documentation.
Source commit id: $head_commit" >> $tmp/commit_msg
commit_status=`${GIT} status -s`

# Make sure that there is something to commit.
# If this is not the case, the documentation is already
# up-to-date and the commit should not be generated.
if test -n "$commit_status"; then
  ${GIT} commit --quiet -F $tmp/commit_msg \
   || abort "failed to generate the git commit"

  echo "* Push the generated commit..."
  ${GIT} push origin gh-pages

  echo "${lgreen}Documentation updated with success!${std}"
else
    notice "Github pages documentation is already up-to-date."
fi

# Clean up the tmp directory
rm -rf $tmp

trap - EXIT
