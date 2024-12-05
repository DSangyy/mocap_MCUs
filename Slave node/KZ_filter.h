#pragma once

# Kolmogorov-Zurbenko filter with k=3, m=5

struct Quat {
  float w;
  float x;
  float y;
  float z;
};

struct KZ_buf {
  Quat k0_buf[1];
  Quat k1_buf[5];
  Quat k2_buf[9];
  Quat k3_buf[13];
};

uint8_t k3_buf_count = 0;
bool is_k3_buf_filled = false;
bool has_initialized = false;


void pop_front(Quat* arr, uint8_t size) {
  for (uint8_t i=0; i<size-1; i++) {
    arr[i] = arr[i+1];
  }
}

void push_back(Quat* arr, uint8_t size, const Quat v) {
  arr[size-1] = v;
}

Quat add(Quat a, Quat b) {
  return Quat{a.w+b.w, a.x+b.x, a.y+b.y, a.z+b.z};
}

Quat div(Quat a, float b) {
  return Quat{a.w/b, a.x/b, a.y/b, a.z/b};
}

float dot(Quat a, Quat b) {
  return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
}

Quat neg(Quat a) {
  return Quat{-a.w, -a.x, -a.y, -a.z};
}

Quat normalize(Quat a, float tolerance=0.00001) {
  float mag2 = pow(a.w,2) + pow(a.x,2) + pow(a.y,2) + pow(a.z,2);
  if (fabs(mag2 - 1.0) > tolerance) {
    float mag = sqrt(mag2);
    a.w /= mag;
    a.x /= mag;
    a.y /= mag;
    a.z /= mag;
  }
  return a;
}

void cal_buf(Quat* buf_in, Quat* buf_out, uint8_t k, uint8_t m, uint8_t start=0) {
  uint8_t size_buf_out = (k-1)*(m-1)+1;
  for (uint8_t i=start; i<size_buf_out; i++) {
      buf_out[i] = Quat{0,0,0,0};
      for (uint8_t j=0; j<m; j++) {
          buf_out[i] = add(buf_out[i], buf_in[i + j]);
      }
      buf_out[i] = div(buf_out[i], m);
  }
}

bool cal_3_5(const Quat q, KZ_buf& KZ_buf, Quat& result) {
  if (!is_k3_buf_filled) {
    k3_buf_count++;
    if (k3_buf_count == 13) {
      is_k3_buf_filled = true;
    }
  }
  
  if (k3_buf_count >= 2 && dot(KZ_buf.k3_buf[k3_buf_count-2], q) < 0) {
    KZ_buf.k3_buf[k3_buf_count-1] = neg(q);
  }
  else {
    KZ_buf.k3_buf[k3_buf_count-1] = q;
  }

  if (!is_k3_buf_filled) {
    return false;
  }

  if (!has_initialized) {
    cal_buf(KZ_buf.k3_buf, KZ_buf.k2_buf, 3, 5);
    cal_buf(KZ_buf.k2_buf, KZ_buf.k1_buf, 2, 5);
    cal_buf(KZ_buf.k1_buf, KZ_buf.k0_buf, 1, 5);
    has_initialized = true;
  }
  else {
    cal_buf(KZ_buf.k3_buf, KZ_buf.k2_buf, 3, 5, 8);
    cal_buf(KZ_buf.k2_buf, KZ_buf.k1_buf, 2, 5, 4);
    cal_buf(KZ_buf.k1_buf, KZ_buf.k0_buf, 1, 5, 0);
  }

  result = normalize(KZ_buf.k0_buf[0]);
  pop_front(KZ_buf.k3_buf, 13);
  pop_front(KZ_buf.k2_buf, 9);
  pop_front(KZ_buf.k1_buf, 5);
  pop_front(KZ_buf.k0_buf, 1);
  return true;
}