#include "../GUI.h"

int main(){
    SC::GUI_base guiBase;
    guiBase.initWindow("Test_window", 1200,800);
    guiBase.run();
    return 0;
}