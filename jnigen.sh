. ../env.sh

javac -h "/home/brahvim/Code/C/SimplePhysicsEngine/include/jni-connector" $PJAVA/src/com/brahvim/physics/*.java
rm $PJAVA/src/com/brahvim/physics/*.class || echo "No \`.class\` files to delete..."

echo "\`$(basename $0)\` run successfully."
