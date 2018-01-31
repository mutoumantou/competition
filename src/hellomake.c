#include <gtk/gtk.h>

static void print_hello (GtkWidget *widget, gpointer   data) {
  g_print ("Hello World\n");
}

static void activate (GtkApplication *app, gpointer user_data) {
  GtkWidget *window;
  GtkWidget *button;
  GtkWidget *button_box;
  GtkWidget *label;

  GtkBuilder *builder;

  builder = gtk_builder_new_from_file ("GUI.glade");
  window  = GTK_WIDGET (gtk_builder_get_object (builder, "mainWindow"));
  gtk_builder_connect_signals (builder, NULL);

/*
  if (window == NULL)
    printf("test failed\n");
  else
    printf("test succeed\n");
*/

  //window = gtk_application_window_new (app);
  //gtk_window_set_title(GTK_WINDOW(window), "Magnet Controller");
  gtk_application_add_window (app, GTK_WINDOW (window));
/*
  window = gtk_application_window_new (app);
  gtk_window_set_title (GTK_WINDOW (window), "WindowHaHa");
  gtk_window_set_default_size (GTK_WINDOW (window), 600, 200);

  //label = gtk_label_new ("This is a test");
  //gtk_container_add (GTK_CONTAINER (window), label);
  //gtk_widget_show (label);

  button_box = gtk_button_box_new (GTK_ORIENTATION_HORIZONTAL);
  gtk_container_add (GTK_CONTAINER (window), button_box);


  button = gtk_button_new_with_label ("Test");
  g_signal_connect (button, "clicked", G_CALLBACK (print_hello), NULL);
  g_signal_connect_swapped (button, "clicked", G_CALLBACK (gtk_widget_destroy), window);
  gtk_container_add (GTK_CONTAINER (button_box), button);

*/
  gtk_widget_show (window);
  //gtk_widget_show (button_box);
  //gtk_widget_show (button);
}

int main (int    argc, char **argv) {
  GtkApplication *app;
  int status;

  app = gtk_application_new ("ca.utoronto.mie.microrobotics", G_APPLICATION_FLAGS_NONE);  // create a new GTKApplication instance
  g_signal_connect (app, "activate", G_CALLBACK (activate), NULL);  // connect a GCallback function to a signal for a particular object
  status = g_application_run (G_APPLICATION (app), argc, argv);     // runs the application
  g_object_unref (app);             // Decreases the reference count of object

  return status;
}
