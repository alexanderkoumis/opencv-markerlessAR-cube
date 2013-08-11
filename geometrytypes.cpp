#include "geometrytypes.h"

matrix44 matrix44::gettransposed() const
{
	matrix44 t;
	for (int i=0; i<4; i++)
		for (int j=0; j<4; j++)
			t.mat[i][j] = mat[j][i];
	return t;
}


matrix44 matrix44::identity()
{
  matrix44 eye;
  
  for (int i=0;i<4; i++)
    for (int j=0;j<4;j++)
      eye.mat[i][j] = i == j ? 1 : 0;
  
  return eye;
}

matrix44 matrix44::getinvertedrt() const
{
  matrix44 t = identity();
  
  for (int col=0;col<3; col++)
  {
    for (int row=0;row<3;row++)
    {
      // Transpose rotation component (inversion)
      t.mat[row][col] = mat[col][row];
    }
    
    // Inverse translation component
    t.mat[3][col] = - mat[3][col];
  }
  return t;
}

matrix33 matrix33::identity()
{
  matrix33 eye;
  
  for (int i=0;i<3; i++)
    for (int j=0;j<3;j++)
      eye.mat[i][j] = i == j ? 1 : 0;
  
  return eye;
}

matrix33 matrix33::gettransposed() const
{
  matrix33 t;
  
  for (int i=0;i<3; i++)
    for (int j=0;j<3;j++)
      t.mat[i][j] = mat[j][i];
  
  return t;
}

vector3 vector3::zero()
{
  vector3 v = { 0,0,0 };
  return v;
}

vector3 vector3::operator-() const
{
  vector3 v = { -data[0],-data[1],-data[2] };
  return v;
}

transformation::transformation() : mrotation(matrix33::identity()), mtranslation(vector3::zero())
{
  
}

transformation::transformation(const matrix33& r, const vector3& t) : mrotation(r), mtranslation(t)
{
  
}

matrix33& transformation::r()
{
  return mrotation;
}

vector3& transformation::t()
{
  return mtranslation;
}

const matrix33& transformation::r() const
{
  return mrotation;
}

const vector3& transformation::t() const
{
  return mtranslation;
}

matrix44 transformation::getmat44() const
{
  matrix44 res = matrix44::identity();
  
  for (int col=0;col<3;col++)
  {
    for (int row=0;row<3;row++)
    {
      // Copy rotation component
      res.mat[row][col] = mrotation.mat[row][col];
    }
    
    // Copy translation component
    res.mat[3][col] = mtranslation.data[col];
  }
  
  return res;
}

transformation transformation::getinverted() const
{
  return transformation(mrotation.gettransposed(), -mtranslation);
}