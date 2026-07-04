# MathLib

ヘッダオンリー C++20 数学ライブラリ。リアルタイム音声処理・3DCG（JUCE）向けに設計。

全 14 モジュールが Linux (GCC) / Windows (MSVC, MinGW) でテスト済みです。

## 提供する型

### ベクトル・行列・回転

| 型 | 説明 |
|---|---|
| `Float2/3/4` | float ベクトル |
| `Double2/3/4` | double ベクトル |
| `Int2/3/4` / `UInt2/3/4` | 整数ベクトル（ビット演算対応）|
| `Bool2/3/4` | マスク・条件分岐用 |
| `Vector2/3/4` | 幾何ベクトル（dot/normalize/cross の対象）|
| `Matrix<T,R,C>` | 任意サイズ行列（列優先）|
| `Matrix2x2/3x3/4x4` | float 正方行列（逆行列・特殊変換付き）|
| `Quaternion` | 単位四元数（`(w,x,y,z)` 順）|
| `Transform` | TRS（平行移動・回転・スケール）変換 |

### 3DCG プリミティブ

| 型 | 説明 |
|---|---|
| `Ray` / `Plane` / `AABB` / `Sphere` | 基本ジオメトリ、交差判定・距離計算付き |
| `Frustum` | 視錐台（6平面）、AABB/Sphere/点のカリング判定 |
| `Triangle` / `Mesh` | 三角形・三角形メッシュ、Ray交差（Moller-Trumbore）・最近接点 |
| `OBB` | 有向境界ボックス、SAT による厳密衝突判定 |

### ユーティリティ

| モジュール | 内容 |
|---|---|
| `MathUtils.h` | clamp/lerp/smoothstep/remap、角度変換、dB変換、MIDI変換、イージング関数 |

## 使い方

```cpp
#include "MathLib/Math.h"   // これ 1 行で全型・全演算が使える

// ベクトル演算
Vector3 a{1, 0, 0}, b{0, 1, 0}, c;
cross(a, b, c);                          // c = {0, 0, 1}
float d = dot(a, b);                     // d = 0.0f

// 行列変換
Matrix4x4 T = Matrix4x4::translation(1, 2, 3);
Matrix4x4 R = Matrix4x4::rotationY(3.14159f * 0.5f);
Matrix4x4 M = T * R;
Vector3 p = M.transformPoint(Vector3{0, 0, 0});

// 四元数
Quaternion q = Quaternion::fromAxisAngle(Vector3{0, 1, 0}, 3.14159f * 0.5f);
Vector3 rotated = q.rotate(Vector3{1, 0, 0});  // {0, 0, -1}
Quaternion q2 = Quaternion::slerp(Quaternion::identity(), q, 0.5f);

// TRS Transform
Transform t;
t.position = {1, 2, 3};
t.rotation = q;
Matrix4x4 worldMatrix = t.toMatrix4x4();

// 視錐台カリング
Frustum frustum = Frustum::fromMatrix(proj * view);
if (frustum.intersects(objectAABB)) { /* draw */ }

// 三角形・Ray交差
Triangle tri{ {0,0,0}, {1,0,0}, {0,1,0} };
Ray ray{ {0.2f,0.2f,5.f}, {0,0,-1} };
float t_hit;
if (intersect(ray, tri, t_hit)) { /* hit at ray.at(t_hit) */ }

// OBB 衝突判定（SAT）
OBB boxA{ {0,0,0}, {1,1,1}, Quaternion::identity() };
OBB boxB{ {1.5f,0,0}, {1,1,1}, Quaternion::fromAxisAngle({0,1,0}, 0.3f) };
if (intersect(boxA, boxB)) { /* collision */ }
```

実際に動く統合デモは `MathLib_demo.cpp` を参照してください（3D 音響シナリオを想定した9セクション構成）。

```bash
g++ -std=c++20 -O2 -mavx2 -I include -o MathLib_demo MathLib_demo.cpp
./MathLib_demo
```

## CMake での組み込み

### A. add_subdirectory（推奨・サブモジュール）

```cmake
# CMakeLists.txt
add_subdirectory(libs/MathLib)

# JUCE プラグインへのリンク
target_link_libraries(MyPlugin
    PRIVATE
        MathLib::MathLib
)
```

ディレクトリ構成:
```
MyPlugin/
├── CMakeLists.txt
├── Source/
│   └── PluginProcessor.cpp
└── libs/
    └── MathLib/          ← git submodule or コピー
        ├── CMakeLists.txt
        └── include/
            └── MathLib/
                └── Math.h  ...
```

### B. FetchContent

```cmake
include(FetchContent)
FetchContent_Declare(
    MathLib
    GIT_REPOSITORY https://github.com/yourname/MathLib.git
    GIT_TAG        v1.0.0
)
FetchContent_MakeAvailable(MathLib)

target_link_libraries(MyPlugin PRIVATE MathLib::MathLib)
```

### C. find_package（インストール済みの場合）

```bash
cmake -B build -DMATHLIB_BUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build
cmake --install build
```

```cmake
find_package(MathLib REQUIRED)
target_link_libraries(MyPlugin PRIVATE MathLib::MathLib)
```

## ビルドオプション

| オプション | デフォルト | 説明 |
|---|---|---|
| `MATHLIB_BUILD_TESTS` | `OFF` | テストをビルドする |
| `MATHLIB_ENABLE_AVX2` | `ON` | `-mavx2` / `/arch:AVX2` を有効化 |
| `MATHLIB_INSTALL` | `ON` | install ルールを生成 |

## テスト

14個のテストファイルが用意されています。

```bash
cmake -B build -DMATHLIB_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

| テスト | 対象 |
|---|---|
| `test_all` / `test_bitop` / `test_vecmath` / `test_vector2` / `test_vector34` | VecN/VecTypes/VecMath |
| `test_matn` | MatrixN/MatrixMul |
| `test_quaternion` | Quaternion |
| `test_transform` | Transform |
| `test_mathutils` | MathUtils |
| `test_geometry` | Ray/Plane/AABB/Sphere |
| `test_frustum` | Frustum |
| `test_triangle` | Triangle/Mesh |
| `test_obb` | OBB/SAT |
| `test_math_h` | Math.h 統合 |

## コンパイラ要件

| 項目 | 要件 |
|---|---|
| C++ 規格 | C++20 以上 |
| 確認済みコンパイラ | GCC 12+ / MSVC 2022+ / MinGW-w64 (w64devkit) |
| 推奨 CPU | AVX2 対応（Haswell 以降）|

## 設計上のポイント

- **参照メンバーなし** — `data[]` 直接アクセスで MSVC の ABI 変換オーバーヘッド（`.isra.0`）を回避
- **列優先（col-major）** — OpenGL / Vulkan / JUCE の `glUniformMatrix4fv` にそのまま渡せる
- **`if constexpr` 展開** — `Matrix4x4 × Matrix4x4` がループゼロの `vperm2f128 + vmulps ymm` 展開
- **`Quaternion` は `(w,x,y,z)` 順** — スカラー部が先頭・`identity()` が `data={1,0,0,0}`
- **グローバル名前空間** — JUCE プラグインのコードで `using namespace` なしで直接使える設計
- **ASCII のみ** — MSVC のコードページ問題（C4819）を避けるため全ファイル ASCII で統一

## ライセンス

MIT License — 詳細は [LICENSE](LICENSE) を参照。
