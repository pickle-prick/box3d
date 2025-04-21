use bevy::prelude::*;
use std::cmp;
use std::ops;
use super::helper::*;

pub struct Mat
{
  pub i_dim: usize,
  pub j_dim: usize,
  pub rows: Vec<Vec<f32>>,
}

impl ops::Index<usize> for Mat
{
  type Output = Vec<f32>;
  fn index(&self, index: usize) -> &Self::Output
  {
    &self.rows[index]
  }
}

impl ops::IndexMut<usize> for Mat
{
  fn index_mut(&mut self, index: usize) -> &mut Self::Output
  {
    &mut self.rows[index]
  }
}

impl cmp::PartialEq for Mat
{
  fn eq(&self, other: &Self) -> bool
  {
    let mut ret = true;
    debug_assert!(self.i_dim == other.i_dim);
    debug_assert!(self.j_dim == other.j_dim);

    'outer: for i in 0..self.i_dim
    {
      for j in 0..self.j_dim
      {
        if (self[i][j]-other[i][j]).abs() > 0.001
        {
          ret = false;
          break 'outer;
        }
      }
    }
    return ret;
  }
}

impl ops::Mul<&Vec<f32>> for &Mat
{
  type Output = Vec<f32>;
  fn mul(self, rhs: &Vec<f32>) -> Vec<f32>
  {
    // row major
    debug_assert_eq!(self.j_dim, rhs.len());
    let mut ret: Vec<f32> = vec![0.0; self.i_dim];

    for i in 0..self.i_dim
    {
      let mut acc: f32 = 0.0;
      for j in 0..self.j_dim
      {
        acc += self[i][j] * rhs[j];
      }
      ret[i] = acc;
    }
    return ret;
  }
}

impl ops::Mul<&Mat> for &Mat
{
  type Output = Mat;
  fn mul(self, rhs: &Mat) -> Mat
  {
    debug_assert_eq!(self.j_dim, rhs.i_dim);
    let mut ret: Mat = Mat::from_dim(self.i_dim, rhs.j_dim);
    for i in 0..self.i_dim
    {
      for j in 0..rhs.j_dim
      {
        let mut acc: f32 = 0.0;
        for k in 0..self.j_dim
        {
          acc += self[i][k] * rhs[k][j];
        }
        ret[i][j] = acc;
      }
    }
    return ret;
  }
}

impl Mat
{
  pub fn tranpose(&self) -> Mat
  {
    let i_dim = self.j_dim;
    let j_dim = self.i_dim;
    let mut ret: Mat = Mat::from_dim(i_dim, j_dim);

    for i in 0..i_dim
    {
      for j in 0..j_dim
      {
        ret[i][j] = self[j][i];
      }
    }
    return ret;
  }

  pub fn swap(&mut self, a: (usize, usize), b: (usize, usize))
  {
    let temp = self[b.0][b.1];
    self[b.0][b.1] = self[a.0][a.1];
    self[a.0][a.1] = temp;
  }

  pub fn from_dim(i_dim: usize, j_dim: usize) -> Self
  {
    let mut rows = vec![vec![0.0; j_dim]; i_dim];
    Self
    {
      i_dim,
      j_dim,
      rows,
    }
  }
}

pub fn gaussj(a: &mut Mat, b: &mut Vec<f32>) -> ()
{
  let n = a.i_dim;
  assert_eq!(a.i_dim, a.j_dim); /* expecting identity matrix */

  let mut icol: usize = 0;
  let mut irow: usize = 0;

  let mut big: f32 = 0.0;
  let mut dum: f32 = 0.0;
  let mut pivinv: f32 = 0.0;
  let mut temp: f32 = 0.0;

  let mut indxc: Vec<i32> = vec![0; n];
  let mut indxr: Vec<i32> = vec![0; n];
  let mut ipiv: Vec<i32> = vec![0; n];

  for i in 0..n
  {
    big = 0.0;
    for j in 0..n
    {
      if ipiv[j] != 1
      {
        for k in 0..n
        {
          if(ipiv[k] == 0)
          {
            if a[j][k].abs() >= big
            {
              big = a[j][k].abs();
              irow = j;
              icol = k;
            }
          }
        }
      }
    }
    ipiv[icol] += 1;

    if(irow != icol)
    {
      for l in 0..n
      {
        a.swap((irow, l), (icol, l));
      }
      b.swap(irow, icol);
    }
    indxr[i] = irow as i32;
    indxc[i] = icol as i32;
    if(a[icol][icol] == 0.0)
    {
      // singular matrix ?
      assert!(false);
    }
    pivinv = 1.0/a[icol][icol];
    a[icol][icol] = 1.0;
    for l in 0..n
    {
      a[icol][l] *= pivinv;
    }
    b[icol] *= pivinv;
    for ll in 0..n
    {
      if(ll != icol)
      {
        dum = a[ll][icol];
        a[ll][icol] = 0.0;
        for l in 0..n
        {
          a[ll][l] -= a[icol][l]*dum;
        }
        b[ll] -= b[icol]*dum;
      }
    }
  }

  // for(l = n-1; l >= 0; l--)
  for l in (0..n).rev()
  {
    if(indxr[l] != indxc[l])
    {
      for k in 0..n
      {
        a.swap((k, indxr[l] as usize), (k, indxc[l] as usize));
      }
    }
  }
}


#[cfg(test)]
mod tests
{
  use super::*;

  fn assert_f32_equal(a: f32, b: f32, epslion: f32)
  {
    assert!((a-b).abs() < epslion);
  }

  #[test]
  fn test_gaussj()
  {
    let mut a = Mat::from_dim(3,3);
    a[0][0] = 2.0;
    a[1][1] = 2.0;
    a[2][2] = 2.0;
    let mut b = vec![1.0; 3];
    gaussj(&mut a, &mut b);
    for (i, v) in vec![0.5, 0.5, 0.5].into_iter().enumerate()
    {
      assert_f32_equal(b[i], v, 0.001);
    }

    let mut a = Mat::from_dim(3,3);
    a[0][0] = 1.0;
    a[0][1] = 3.0;
    a[0][2] = 2.0;
    a[1][1] = 4.0;
    a[2][0] = 5.0;
    a[2][2] = 6.0;
    let mut b = vec![11.0, 4.0, 28.0];
    gaussj(&mut a, &mut b);
    for (i, v) in vec![2.0, 1.0, 3.0].into_iter().enumerate()
    {
      assert_f32_equal(b[i], v, 0.001);
    }

    let mut a = Mat::from_dim(3,3);
    a[0][0] = 1.0;
    a[0][2] = 5.0;
    a[1][0] = 3.0;
    a[1][1] = 4.0;
    a[2][0] = 2.0;
    a[2][2] = 6.0;
    let mut b = vec![17.0, 10.0, 22.0];
    gaussj(&mut a, &mut b);
    for (i, v) in vec![2.0, 1.0, 3.0].into_iter().enumerate()
    {
      assert_f32_equal(b[i], v, 0.001);
    }
  }

  #[test]
  fn test_Mat()
  {
    // test swap
    let mut a = Mat::from_dim(3,3);
    a[0][0] = 1.0;
    a[0][1] = 3.0;
    a[0][2] = 2.0;
    a[1][1] = 4.0;
    a[2][0] = 5.0;
    a[2][2] = 6.0;
    a.swap((0,0), (1,1));
    assert_eq!(a[0][0], 4.0);
    assert_eq!(a[1][1], 1.0);

    // test Mat*Mat
    let mut a = Mat::from_dim(2,3);
    a[0] = vec![1.0, 2.0, 3.0];
    a[1] = vec![4.0, 5.0, 6.0];

    let mut b = Mat::from_dim(3,2);
    b[0] = vec![7.0, 8.0];
    b[1] = vec![9.0, 10.0];
    b[2] = vec![11.0, 12.0];

    let c = &a*&b;
    assert_eq!(c[0][0], 58.0);
    assert_eq!(c[0][1], 64.0);
    assert_eq!(c[1][0], 139.0);
    assert_eq!(c[1][1], 154.0);

    // test eq
    let mut a = Mat::from_dim(2,3);
    a[0] = vec![1.0, 2.0, 3.0];
    a[1] = vec![4.0, 5.0, 6.0];
    let mut b = Mat::from_dim(2,3);
    b[0] = vec![1.0, 2.0, 3.0];
    b[1] = vec![4.0, 5.0, 6.0];
    assert!(a == b);
    b[0][0] = 0.0;
    assert!(a != b);

    // test transpose
    let mut a = Mat::from_dim(2,3);
    a[0] = vec![1.0, 2.0, 3.0];
    a[1] = vec![4.0, 5.0, 6.0];
    let mut b = Mat::from_dim(3,2);
    b[0] = vec![1.0, 4.0];
    b[1] = vec![2.0, 5.0];
    b[2] = vec![3.0, 6.0];
    assert!(a.tranpose() == b);

    // test Mat * Vec
    let mut a = Mat::from_dim(3,3);
    a[0][0] = 1.0;
    a[0][1] = 3.0;
    a[0][2] = 2.0;
    a[1][1] = 4.0;
    a[2][0] = 5.0;
    a[2][2] = 6.0;
    let mut b = vec![2.0, 1.0, 3.0];
    let mut c = &a * &b;
    gaussj(&mut a, &mut b);
    for (i, v) in vec![11.0, 4.0, 28.0].into_iter().enumerate()
    {
      assert_f32_equal(v, c[i], 0.001);
    }
  }
}
