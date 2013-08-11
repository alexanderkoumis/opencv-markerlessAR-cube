#ifndef geometrytypes_hpp
#define geometrytypes_hpp

struct matrix44
{
	union
	{
		float data[16];
		float mat[4][4];
	};
	matrix44 gettransposed() const;
	matrix44 getinvertedrt() const;
	static matrix44 identity();
};

struct matrix33
{
  union
  {
    float data[9];
    float mat[3][3];
  };
  
  static matrix33 identity();
  matrix33 gettransposed() const;
};

struct vector4
{
  float data[4];
};

struct vector3
{
  float data[3];
  
  static vector3 zero();
  vector3 operator-() const;
};

struct transformation
{
	transformation();
	transformation(const matrix33& r, const vector3& t);

	matrix33& r();
	vector3& t();

	const matrix33& r() const;
	const vector3& t() const;

	matrix44 getmat44() const;

	transformation getinverted() const;
private:
	matrix33 mrotation;
	vector3 mtranslation;
};

#endif