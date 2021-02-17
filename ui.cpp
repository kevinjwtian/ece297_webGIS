#include <ui.h>

void help_info(GtkWidget *widget, ezgl::application *application){
    (void) widget;
    
    GObject *help = application->get_object("helpRevealer");
    gtk_revealer_set_reveal_child ((GtkRevealer*)help,TRUE);

}
