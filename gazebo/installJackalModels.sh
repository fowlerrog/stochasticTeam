# clones jackal repo and edits urdf files
read -p "### This script will clone a jackal_description repo. [y/n] " input
if ! [[ $input =~ ^[yY]$ ]] ; then
	echo "Canceling"
	exit 0
fi

# save this script's path
SCRIPT=$(realpath "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(dirname "$SCRIPT")
echo "    This script is at $SCRIPT_DIR"

# get jackal models
echo "### Cloning jackal for jackal_description ###"
mkdir -p $SCRIPT_DIR/resources/jackal_repo
cd $SCRIPT_DIR/resources/jackal_repo
git clone https://github.com/jackal/jackal.git

# copy urdfs and meshes to resources folder
cd $SCRIPT_DIR/resources
mkdir -p jackal_description/urdf
mkdir -p jackal_description/meshes
cp -r jackal_repo/jackal/jackal_description/meshes/* jackal_description/meshes/
cp -r jackal_repo/jackal/jackal_description/urdf/* jackal_description/urdf/

# also replace any mention of jackal package in urdfs
cd jackal_description/urdf
JACKAL_DIR_FILE="file://$SCRIPT_DIR/resources/jackal_description"
JACKAL_DIR_FIND="$SCRIPT_DIR/resources/jackal_description"
echo "Replacing in each file:"
echo "    'package://jackal_description' with '$JACKAL_DIR_FILE'"
echo "    '\$(find jackal_description)' with '$JACKAL_DIR_FIND'"
echo "    Also removing all lines with \$(find package) where package != jackal_description"
for file in $(find . -type f -iname "*.urdf.xacro"); do
    echo "        $file"
    # 's/a/b/g' replaces a with b globally, '/s/d removes all lines containing s'
    # -i flag replaces file in place, -E flag enables regex
    SAFE_JACKAL_DIR_FILE="${JACKAL_DIR_FILE//\//\\/}" # escape slashes for regex usage
    SAFE_JACKAL_DIR_FIND="${JACKAL_DIR_FIND//\//\\/}"
    sed -i -E "s/package:\/\/jackal_description/$SAFE_JACKAL_DIR_FILE/g; s/\\\$\(find jackal_description\)/$SAFE_JACKAL_DIR_FIND/g; /\\\$\(find [^j]*\)/d" $file
done 
