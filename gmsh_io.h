#pragma once

# include <cstdlib>
# include <iostream>
# include <iomanip>
# include <fstream>
# include <ctime>
# include <cmath>
# include <cstring>
# include <string>

class gmsh_io
{
public:
  static char ch_cap ( char ch );
  static bool ch_eqi ( char ch1, char ch2 );
  static int ch_to_digit ( char ch );
  static void gmsh_data_read ( std::string gmsh_filename, int node_dim, int node_num, double node_x[], int element_order, int element_num, int element_node[] );
  static void gmsh_size_read ( std::string gmsh_filename, int &node_num, int &node_dim, int &element_num, int &element_order );
  static int *gmsh_mesh2d_element_data_example ( int element_num, int element_order );
  static void gmsh_mesh2d_element_size_example ( int &element_num, int &element_order );
  static double *gmsh_mesh2d_node_data_example ( int node_num, int node_dim );
  static void gmsh_mesh2d_node_size_example ( int &node_num, int &node_dim );
  static void gmsh_mesh1d_write ( std::string gmsh_filename, int m, int node_num, double node_x[], int element_order, int element_num, int element_node[] );
  static void gmsh_mesh2d_write ( std::string gmsh_filename, int m, int node_num, double node_x[], int element_order, int element_num, int element_node[] );
  static void gmsh_mesh3d_write ( std::string gmsh_filename, int m, int node_num, double node_x[], int element_order, int element_num, int element_node[] );
  static int *i4mat_copy_new ( int m, int n, int a1[] );
  static void i4mat_transpose_print ( int m, int n, int a[], std::string title );
  static void i4mat_transpose_print_some ( int m, int n, int a[], int ilo, int jlo, int ihi, int jhi, std::string title );
  static void mesh_base_one ( int node_num, int element_order, int element_num, int element_node[] );
  static double r8_max ( double x, double y );
  static double r8_min ( double x, double y );
  static double *r8mat_copy_new ( int m, int n, double a1[] );
  static void r8mat_transpose_print ( int m, int n, double a[], std::string title );
  static void r8mat_transpose_print_some ( int m, int n, double a[], int ilo, int jlo, int ihi, int jhi, std::string title );
  static bool s_begin ( std::string s1, std::string s2 );
  static int s_len_trim ( std::string s );
  static int s_to_i4 ( std::string s, int &last, bool &error );
  static double s_to_r8 ( std::string s, int &lchar, bool &error );
  static void timestamp ( );
};
