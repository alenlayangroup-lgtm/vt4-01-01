// Halden VT-4 — circuit engine.
//
// A fixed-capacity netlist (no heap, trivially copyable) that is compiled into
// a discrete-time state-space model with the trapezoidal rule ("nodal DK"
// method). Supported elements:
//
//   R, C, L                passive parts (C and L carry one state each)
//   V  (a, b, input k)     voltage source  v(a) − v(b) = u[k]
//   I  (a, b, input k)     current source  u[k] flows a → b through the source
//   E  (op, on, cp, cn, g) voltage-controlled voltage source (ideal pad tap,
//                          cathode-follower small-signal model, …)
//   G  (a, b, cp, cn, g)   voltage-controlled current source (linearised tubes)
//   N  (a, b, port m)      nonlinear current i[m] flowing a → b (tube plate /
//                          grid current, rectifier current); solved per sample
//                          by the owner of the circuit
//
// After compile() the circuit runs as
//
//     p  = D·s + E·u              (controlling voltages of nonlinear ports)
//     i  = f(p + F·i)             (solved by the caller)
//     y  = Oy·s + Ou·u + Oi·i
//     s' = Φ·s + Γ·u + Λ·i
//
// Each reactive element can be prewarped to its own frequency: the
// trapezoidal integrator constant 2/T is replaced by ω/tan(ωT/2), optionally
// skewed (see Elem::skew). Every element stays a passive discrete one-port
// for any positive constant, so the discretised network inherits the
// stability of the analog one.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <vector>

namespace vt4
{

constexpr double kPi = 3.14159265358979323846;

struct CircuitWorkspace;

class Circuit
{
public:
    static constexpr int kGnd = -1;
    static constexpr int kMaxNodes = 48;
    static constexpr int kMaxElems = 96;
    static constexpr int kMaxStates = 16;
    static constexpr int kMaxInputs = 8;
    static constexpr int kMaxNL = 2;
    static constexpr int kMaxCtrl = 2;
    static constexpr int kMaxOut = 12;
    static constexpr int kMaxExtra = 16;                      // V + E rows
    static constexpr int kMaxM = kMaxNodes + kMaxExtra;
    static constexpr int kMaxCols = kMaxStates + kMaxInputs + kMaxNL;

    enum class Kind : uint8_t { R, C, L, V, I, E, G, N };

    struct Elem
    {
        Kind kind;
        int8_t a, b, c, d;   // nodes (c, d = controlling pair for E/G)
        int8_t idx;          // input index (V, I) or port index (N)
        double value;        // ohms / farads / henries / gain
        double warp;         // prewarp frequency in Hz for C/L (0 = plain 2/T)
        int16_t tag;         // identity of a reactive part across recompiles (−1: by order)
        double skew = 0.0;   // c ← c·(1 − skew·θ²), θ = π·warp/fs (see compile)
    };

    // ---- netlist ----------------------------------------------------------
    void clear() noexcept { nNodes = 0; nElems = 0; nCtrl = 0; nOut = 0; nInputs = 0; nNL = 0; }
    int addNode() noexcept { return nNodes < kMaxNodes ? nNodes++ : kMaxNodes - 1; }

    void R (int a, int b, double ohms)                 { add ({ Kind::R, (int8_t) a, (int8_t) b, 0, 0, 0, std::max (ohms, 1.0e-6), 0.0, -1 }); }
    void C (int a, int b, double farads, double w = 0, int tag = -1, double skew = 0.0)  { add ({ Kind::C, (int8_t) a, (int8_t) b, 0, 0, 0, farads, w, (int16_t) tag, skew }); }
    void L (int a, int b, double henries, double w = 0, int tag = -1, double skew = 0.0) { add ({ Kind::L, (int8_t) a, (int8_t) b, 0, 0, 0, henries, w, (int16_t) tag, skew }); }
    void V (int a, int b, int input)                   { add ({ Kind::V, (int8_t) a, (int8_t) b, 0, 0, (int8_t) input, 0.0, 0.0, -1 }); nInputs = std::max (nInputs, input + 1); }
    void I (int a, int b, int input)                   { add ({ Kind::I, (int8_t) a, (int8_t) b, 0, 0, (int8_t) input, 0.0, 0.0, -1 }); nInputs = std::max (nInputs, input + 1); }
    void E (int op, int on, int cp, int cn, double g)  { add ({ Kind::E, (int8_t) op, (int8_t) on, (int8_t) cp, (int8_t) cn, 0, g, 0.0, -1 }); }
    void G (int a, int b, int cp, int cn, double g)    { add ({ Kind::G, (int8_t) a, (int8_t) b, (int8_t) cp, (int8_t) cn, 0, g, 0.0, -1 }); }
    int  N (int a, int b)                              { add ({ Kind::N, (int8_t) a, (int8_t) b, 0, 0, (int8_t) nNL, 0.0, 0.0, -1 }); return nNL++; }

    int control (int p, int n) noexcept { ctrlNodes[(size_t) nCtrl] = { (int8_t) p, (int8_t) n }; return nCtrl++; }
    int output  (int p, int n = kGnd) noexcept { outNodes[(size_t) nOut] = { (int8_t) p, (int8_t) n }; return nOut++; }

    int numStates() const noexcept
    {
        int k = 0;
        for (int e = 0; e < nElems; ++e)
            if (elems[(size_t) e].kind == Kind::C || elems[(size_t) e].kind == Kind::L) ++k;
        return k;
    }
    int numInputs() const noexcept { return nInputs; }
    int numNL() const noexcept { return nNL; }
    int states() const noexcept { return ns; }

    // ---- compile ----------------------------------------------------------
    // Builds the discrete model. If the number of states is unchanged the
    // state vector is kept, so element values can be changed while running.
    bool compile (double sampleRate, CircuitWorkspace& ws);

    void resetState() noexcept { s.fill (0.0); sPrev.fill (0.0); }
    // sets a state as a DC (steady) value
    void setState (int k, double v) noexcept { s[(size_t) k] = v; sPrev[(size_t) k] = v; }
    double state (int k) const noexcept { return s[(size_t) k]; }

    // ---- run --------------------------------------------------------------
    // Linear circuits (no nonlinear ports): one output, inputs u[0..nu).
    template <int NS>
    inline double stepLinearT (const double* u) noexcept
    {
        double y = 0.0;
        for (int k = 0; k < nu; ++k) y += Ou[0][(size_t) k] * u[k];
        for (int j = 0; j < NS; ++j) y += Oy[0][(size_t) j] * s[(size_t) j];
        double n[NS > 0 ? NS : 1];
        for (int e = 0; e < NS; ++e)
        {
            double acc = 0.0;
            for (int k = 0; k < nu; ++k) acc += Gam[(size_t) e][(size_t) k] * u[k];
            for (int j = 0; j < NS; ++j) acc += Phi[(size_t) e][(size_t) j] * s[(size_t) j];
            n[e] = acc;
        }
        for (int e = 0; e < NS; ++e) { sPrev[(size_t) e] = s[(size_t) e]; s[(size_t) e] = n[e]; }
        return y;
    }

    inline double stepLinear (const double* u) noexcept
    {
        switch (ns)
        {
            case 0:  return stepLinearT<0> (u);
            case 1:  return stepLinearT<1> (u);
            case 2:  return stepLinearT<2> (u);
            case 3:  return stepLinearT<3> (u);
            case 4:  return stepLinearT<4> (u);
            case 5:  return stepLinearT<5> (u);
            case 6:  return stepLinearT<6> (u);
            case 7:  return stepLinearT<7> (u);
            case 8:  return stepLinearT<8> (u);
            case 9:  return stepLinearT<9> (u);
            case 10: return stepLinearT<10> (u);
            case 11: return stepLinearT<11> (u);
            case 12: return stepLinearT<12> (u);
            case 13: return stepLinearT<13> (u);
            case 14: return stepLinearT<14> (u);
            case 15: return stepLinearT<15> (u);
            default: return stepLinearT<16> (u);
        }
    }

    // Nonlinear use, split in two phases around the caller's solver.
    inline void controls (const double* u, double* p) const noexcept
    {
        for (int c = 0; c < nc; ++c)
        {
            double acc = 0.0;
            for (int k = 0; k < nu; ++k) acc += Ec[(size_t) c][(size_t) k] * u[k];
            for (int j = 0; j < ns; ++j) acc += Dc[(size_t) c][(size_t) j] * s[(size_t) j];
            p[c] = acc;
        }
    }
    double feedback (int c, int m) const noexcept { return Fc[(size_t) c][(size_t) m]; }

    inline double outputNL (int o, const double* u, const double* i) const noexcept
    {
        double y = 0.0;
        for (int k = 0; k < nu; ++k) y += Ou[(size_t) o][(size_t) k] * u[k];
        for (int j = 0; j < ns; ++j) y += Oy[(size_t) o][(size_t) j] * s[(size_t) j];
        for (int m = 0; m < nnl; ++m) y += Oi[(size_t) o][(size_t) m] * i[m];
        return y;
    }

    inline void advanceNL (const double* u, const double* i) noexcept
    {
        double n[kMaxStates];
        for (int e = 0; e < ns; ++e)
        {
            double acc = 0.0;
            for (int k = 0; k < nu; ++k) acc += Gam[(size_t) e][(size_t) k] * u[k];
            for (int j = 0; j < ns; ++j) acc += Phi[(size_t) e][(size_t) j] * s[(size_t) j];
            for (int m = 0; m < nnl; ++m) acc += Lam[(size_t) e][(size_t) m] * i[m];
            n[e] = acc;
        }
        for (int e = 0; e < ns; ++e) { sPrev[(size_t) e] = s[(size_t) e]; s[(size_t) e] = n[e]; }
    }

    // Fixed-size variants of controls / outputNL / advanceNL for small
    // nonlinear stages (sizes known at compile time → fully unrolled).
    template <int NS, int NU, int NNL, int NC>
    inline void controlsT (const double* u, double* p) const noexcept
    {
        for (int c = 0; c < NC; ++c)
        {
            double acc = 0.0;
            for (int k = 0; k < NU; ++k) acc += Ec[(size_t) c][(size_t) k] * u[k];
            for (int j = 0; j < NS; ++j) acc += Dc[(size_t) c][(size_t) j] * s[(size_t) j];
            p[c] = acc;
        }
    }
    template <int NS, int NU, int NNL>
    inline double outputT (int o, const double* u, const double* i) const noexcept
    {
        double y = 0.0;
        for (int k = 0; k < NU; ++k) y += Ou[(size_t) o][(size_t) k] * u[k];
        for (int j = 0; j < NS; ++j) y += Oy[(size_t) o][(size_t) j] * s[(size_t) j];
        for (int m = 0; m < NNL; ++m) y += Oi[(size_t) o][(size_t) m] * i[m];
        return y;
    }
    template <int NS, int NU, int NNL>
    inline void advanceT (const double* u, const double* i) noexcept
    {
        double n[NS > 0 ? NS : 1];
        for (int e = 0; e < NS; ++e)
        {
            double acc = 0.0;
            for (int k = 0; k < NU; ++k) acc += Gam[(size_t) e][(size_t) k] * u[k];
            for (int j = 0; j < NS; ++j) acc += Phi[(size_t) e][(size_t) j] * s[(size_t) j];
            for (int m = 0; m < NNL; ++m) acc += Lam[(size_t) e][(size_t) m] * i[m];
            n[e] = acc;
        }
        for (int e = 0; e < NS; ++e) { sPrev[(size_t) e] = s[(size_t) e]; s[(size_t) e] = n[e]; }
    }
    int numCtrl() const noexcept { return nc; }

    // Steady state for constant inputs and constant nonlinear currents:
    // s = (I − Φ)⁻¹ (Γu + Λi). Returns false if singular.
    bool steadyState (const double* u, const double* i);

    // DC (static) model of the netlist: capacitors open, inductors shorted.
    // v_ctrl = E·u + F·i, y = Ou·u + Oi·i. Used to find operating points.
    struct StaticModel
    {
        int nu = 0, nnl = 0, nc = 0, no = 0;
        double Ec[kMaxCtrl][kMaxInputs] {}, Fc[kMaxCtrl][kMaxNL] {};
        double Ou[kMaxOut][kMaxInputs] {}, Oi[kMaxOut][kMaxNL] {};
        double feedback (int c, int m) const noexcept { return Fc[c][m]; }
        void controls (const double* u, double* p) const noexcept
        {
            for (int c = 0; c < nc; ++c) { double a = 0.0; for (int k = 0; k < nu; ++k) a += Ec[c][k] * u[k]; p[c] = a; }
        }
        double output (int o, const double* u, const double* i) const noexcept
        {
            double y = 0.0;
            for (int k = 0; k < nu; ++k) y += Ou[o][k] * u[k];
            for (int m = 0; m < nnl; ++m) y += Oi[o][m] * i[m];
            return y;
        }
    };
    bool compileStatic (CircuitWorkspace& ws, StaticModel& sm) const;

    // Discrete small-signal transfer function from input k to output o at
    // frequency f, with the nonlinear ports linearised as i = Jf·v (Jf is
    // nnl×nc, row-major, may be null for linear circuits).
    std::complex<double> digitalResponse (double f, int k = 0, int o = 0, const double* Jf = nullptr) const;

    // Continuous-time transfer function of the netlist (element values as
    // given, nonlinear ports open) from input k to output o.
    std::complex<double> analogResponse (double f, int k = 0, int o = 0) const;

    double sampleRate() const noexcept { return fsCompiled; }
    // changes on every successful compile (cache key for derived models)
    uint32_t version() const noexcept { return compileCount; }

private:
    uint32_t compileCount = 0;
    void add (const Elem& e) noexcept { if (nElems < kMaxElems) elems[(size_t) nElems++] = e; }

    std::array<Elem, kMaxElems> elems {};
    struct Pair { int8_t p, n; };
    std::array<Pair, kMaxCtrl> ctrlNodes {};
    std::array<Pair, kMaxOut> outNodes {};
    int nNodes = 0, nElems = 0, nCtrl = 0, nOut = 0, nInputs = 0, nNL = 0;

    // compiled model
    int ns = 0, nu = 0, nnl = 0, nc = 0, no = 0;
    double fsCompiled = 0.0;
    std::array<std::array<double, kMaxStates>, kMaxStates> Phi {};
    std::array<std::array<double, kMaxInputs>, kMaxStates> Gam {};
    std::array<std::array<double, kMaxNL>, kMaxStates> Lam {};
    std::array<std::array<double, kMaxStates>, kMaxCtrl> Dc {};
    std::array<std::array<double, kMaxInputs>, kMaxCtrl> Ec {};
    std::array<std::array<double, kMaxNL>, kMaxCtrl> Fc {};
    std::array<std::array<double, kMaxStates>, kMaxOut> Oy {};
    std::array<std::array<double, kMaxInputs>, kMaxOut> Ou {};
    std::array<std::array<double, kMaxNL>, kMaxOut> Oi {};
    std::array<double, kMaxStates> s {}, sPrev {};
    // identity of each state's part (for state transfer on recompile)
    std::array<int, kMaxStates> stTag {};
    std::array<bool, kMaxStates> stIsC {};
    std::array<double, kMaxStates> stG {};
};

struct CircuitWorkspace
{
    std::array<double, Circuit::kMaxM * Circuit::kMaxM> A {};
    std::array<double, Circuit::kMaxM * Circuit::kMaxCols> Z {};
};

// ---------------------------------------------------------------------------

inline bool Circuit::compile (double sampleRate, CircuitWorkspace& ws)
{
    const double T = 1.0 / sampleRate;
    const int N = nNodes;
    int extra = 0;
    for (int e = 0; e < nElems; ++e)
        if (elems[(size_t) e].kind == Kind::V || elems[(size_t) e].kind == Kind::E) ++extra;
    const int M = N + extra;
    const int nst = numStates();
    if (nst > kMaxStates || M > kMaxM || nInputs > kMaxInputs || nNL > kMaxNL || M <= 0)
        return false;

    const int cols = nst + nInputs + nNL;
    std::fill (ws.A.begin(), ws.A.begin() + M * M, 0.0);
    std::fill (ws.Z.begin(), ws.Z.begin() + M * cols, 0.0);
    auto A = [&] (int r, int c) -> double& { return ws.A[(size_t) (r * M + c)]; };
    auto Z = [&] (int r, int c) -> double& { return ws.Z[(size_t) (r * cols + c)]; };
    auto stampG = [&] (int a, int b, double g)
    {
        if (a >= 0) A (a, a) += g;
        if (b >= 0) A (b, b) += g;
        if (a >= 0 && b >= 0) { A (a, b) -= g; A (b, a) -= g; }
    };

    struct Reactive { int a, b; double gamma, sigma; };
    std::array<Reactive, kMaxStates> rx {};
    std::array<int, kMaxStates> newTag {};
    std::array<bool, kMaxStates> newIsC {};
    int k = 0, row = N;
    for (int e = 0; e < nElems; ++e)
    {
        const auto& el = elems[(size_t) e];
        switch (el.kind)
        {
            case Kind::R: stampG (el.a, el.b, 1.0 / el.value); break;
            case Kind::C:
            case Kind::L:
            {
                double c = 2.0 / T;
                if (el.warp > 0.0)
                {
                    const double w = 2.0 * kPi * std::min (el.warp, 0.45 * sampleRate);
                    c = w / std::tan (0.5 * w * T);
                    // Skewed pair: a resonator whose L and C constants are
                    // spread around the prewarp value (L slightly below, C
                    // slightly above) follows the analog skirts on both sides
                    // of the resonance instead of matching one point only.
                    if (el.skew != 0.0)
                    {
                        const double th = 0.5 * w * T;
                        c *= 1.0 - el.skew * th * th;
                    }
                }
                else if (el.warp < 0.0)
                {
                    // "imaginary" prewarp frequency: c = Ω / tanh(ΩT/2) > 2/T
                    // (the complementary constant of a dual-warped resonator)
                    const double w = 2.0 * kPi * std::min (-el.warp, 4.0 * sampleRate);
                    c = w / std::tanh (0.5 * w * T);
                }
                const bool isC = el.kind == Kind::C;
                const double g = isC ? el.value * c : 1.0 / (el.value * c);
                stampG (el.a, el.b, g);
                // history source: C → +J into a, L → −K into a
                const double sgn = isC ? 1.0 : -1.0;
                if (el.a >= 0) Z (el.a, k) += sgn;
                if (el.b >= 0) Z (el.b, k) -= sgn;
                rx[(size_t) k] = { el.a, el.b, 2.0 * g, isC ? -1.0 : 1.0 };
                newTag[(size_t) k] = el.tag;
                newIsC[(size_t) k] = isC;
                ++k;
                break;
            }
            case Kind::V:
            {
                if (el.a >= 0) { A (el.a, row) += 1.0; A (row, el.a) += 1.0; }
                if (el.b >= 0) { A (el.b, row) -= 1.0; A (row, el.b) -= 1.0; }
                Z (row, nst + el.idx) = 1.0;
                ++row;
                break;
            }
            case Kind::E:
            {
                if (el.a >= 0) { A (el.a, row) += 1.0; A (row, el.a) += 1.0; }
                if (el.b >= 0) { A (el.b, row) -= 1.0; A (row, el.b) -= 1.0; }
                if (el.c >= 0) A (row, el.c) -= el.value;
                if (el.d >= 0) A (row, el.d) += el.value;
                ++row;
                break;
            }
            case Kind::G:
            {
                // current g·(v_c − v_d) leaves a, enters b
                if (el.a >= 0) { if (el.c >= 0) A (el.a, el.c) += el.value; if (el.d >= 0) A (el.a, el.d) -= el.value; }
                if (el.b >= 0) { if (el.c >= 0) A (el.b, el.c) -= el.value; if (el.d >= 0) A (el.b, el.d) += el.value; }
                break;
            }
            case Kind::I:
            {
                if (el.a >= 0) Z (el.a, nst + el.idx) -= 1.0;
                if (el.b >= 0) Z (el.b, nst + el.idx) += 1.0;
                break;
            }
            case Kind::N:
            {
                if (el.a >= 0) Z (el.a, nst + nInputs + el.idx) -= 1.0;
                if (el.b >= 0) Z (el.b, nst + nInputs + el.idx) += 1.0;
                break;
            }
        }
    }

    // Solve A · X = Z (Gaussian elimination with partial pivoting).
    for (int c = 0; c < M; ++c)
    {
        int p = c;
        double best = std::fabs (A (c, c));
        for (int r = c + 1; r < M; ++r)
            if (std::fabs (A (r, c)) > best) { best = std::fabs (A (r, c)); p = r; }
        if (best < 1.0e-300) return false;
        if (p != c)
        {
            for (int j = 0; j < M; ++j) std::swap (A (p, j), A (c, j));
            for (int j = 0; j < cols; ++j) std::swap (Z (p, j), Z (c, j));
        }
        const double inv = 1.0 / A (c, c);
        for (int r = c + 1; r < M; ++r)
        {
            const double f = A (r, c) * inv;
            if (f == 0.0) continue;
            for (int j = c; j < M; ++j) A (r, j) -= f * A (c, j);
            for (int j = 0; j < cols; ++j) Z (r, j) -= f * Z (c, j);
        }
    }
    for (int r = M - 1; r >= 0; --r)
    {
        const double inv = 1.0 / A (r, r);
        for (int j = 0; j < cols; ++j)
        {
            double acc = Z (r, j);
            for (int c = r + 1; c < M; ++c) acc -= A (r, c) * Z (c, j);
            Z (r, j) = acc * inv;
        }
    }

    auto nodeV = [&] (int node, int col) { return node >= 0 ? Z (node, col) : 0.0; };
    // Carry the physical state across the recompile, exactly: with the
    // trapezoidal companion of a capacitor, J[n] = g·v[n−1] + i[n−1] and
    // J[n] + J[n−1] = 2g·v[n−1]; for an inductor K[n] = i[n−1] + g·v[n−1] and
    // K[n] − K[n−1] = 2g·v[n−1]. So the last voltage and current of every
    // part are recovered from the last two states and re-expressed with the
    // new conductance: a capacitor keeps its voltage and current, an
    // inductor its current and voltage. Parts are matched by tag, untagged
    // parts by their order.
    {
        std::array<double, kMaxStates> s2 {}, p2 {};
        const bool compiled = fsCompiled > 0.0;
        int untaggedOld[kMaxStates], nuo = 0;
        for (int e = 0; e < ns; ++e) if (stTag[(size_t) e] < 0) untaggedOld[nuo++] = e;
        int nun = 0;
        for (int e = 0; e < nst; ++e)
        {
            int src = -1;
            if (compiled)
            {
                if (newTag[(size_t) e] >= 0)
                {
                    for (int o = 0; o < ns; ++o)
                        if (stTag[(size_t) o] == newTag[(size_t) e] && stIsC[(size_t) o] == newIsC[(size_t) e]) { src = o; break; }
                }
                else
                {
                    if (nun < nuo && stIsC[(size_t) untaggedOld[nun]] == newIsC[(size_t) e]) src = untaggedOld[nun];
                    ++nun;
                }
            }
            if (src < 0 || ! (stG[(size_t) src] > 0.0)) continue;
            const double go = stG[(size_t) src], gn = 0.5 * rx[(size_t) e].gamma;
            const double a = s[(size_t) src], b = sPrev[(size_t) src];
            if (newIsC[(size_t) e])
            {
                const double v = (a + b) / (2.0 * go), i = a - go * v;
                s2[(size_t) e] = gn * v + i;
                p2[(size_t) e] = 2.0 * gn * v - s2[(size_t) e];
            }
            else
            {
                const double v = (a - b) / (2.0 * go), i = 0.5 * (a + b);
                s2[(size_t) e] = i + gn * v;
                p2[(size_t) e] = s2[(size_t) e] - 2.0 * gn * v;
            }
        }
        s = s2; sPrev = p2;
        for (int e = 0; e < nst; ++e)
        {
            stTag[(size_t) e] = newTag[(size_t) e];
            stIsC[(size_t) e] = newIsC[(size_t) e];
            stG[(size_t) e] = 0.5 * rx[(size_t) e].gamma;
        }
    }
    ns = nst; nu = nInputs; nnl = nNL; nc = nCtrl; no = nOut;
    for (int e = 0; e < ns; ++e)
    {
        const auto& r = rx[(size_t) e];
        for (int j = 0; j < ns; ++j)
            Phi[(size_t) e][(size_t) j] = r.gamma * (nodeV (r.a, j) - nodeV (r.b, j)) + (e == j ? r.sigma : 0.0);
        for (int q = 0; q < nu; ++q)
            Gam[(size_t) e][(size_t) q] = r.gamma * (nodeV (r.a, ns + q) - nodeV (r.b, ns + q));
        for (int m = 0; m < nnl; ++m)
            Lam[(size_t) e][(size_t) m] = r.gamma * (nodeV (r.a, ns + nu + m) - nodeV (r.b, ns + nu + m));
    }
    for (int c = 0; c < nc; ++c)
    {
        const auto& pr = ctrlNodes[(size_t) c];
        for (int j = 0; j < ns; ++j)  Dc[(size_t) c][(size_t) j] = nodeV (pr.p, j) - nodeV (pr.n, j);
        for (int q = 0; q < nu; ++q)  Ec[(size_t) c][(size_t) q] = nodeV (pr.p, ns + q) - nodeV (pr.n, ns + q);
        for (int m = 0; m < nnl; ++m) Fc[(size_t) c][(size_t) m] = nodeV (pr.p, ns + nu + m) - nodeV (pr.n, ns + nu + m);
    }
    for (int o = 0; o < no; ++o)
    {
        const auto& pr = outNodes[(size_t) o];
        for (int j = 0; j < ns; ++j)  Oy[(size_t) o][(size_t) j] = nodeV (pr.p, j) - nodeV (pr.n, j);
        for (int q = 0; q < nu; ++q)  Ou[(size_t) o][(size_t) q] = nodeV (pr.p, ns + q) - nodeV (pr.n, ns + q);
        for (int m = 0; m < nnl; ++m) Oi[(size_t) o][(size_t) m] = nodeV (pr.p, ns + nu + m) - nodeV (pr.n, ns + nu + m);
    }
    fsCompiled = sampleRate;
    ++compileCount;
    return true;
}

inline bool Circuit::compileStatic (CircuitWorkspace& ws, StaticModel& sm) const
{
    const int N = nNodes;
    int extra = 0;
    for (int e = 0; e < nElems; ++e)
        if (elems[(size_t) e].kind == Kind::V || elems[(size_t) e].kind == Kind::E) ++extra;
    const int M = N + extra;
    const int cols = nInputs + nNL;
    if (M > kMaxM || M <= 0) return false;
    std::fill (ws.A.begin(), ws.A.begin() + M * M, 0.0);
    std::fill (ws.Z.begin(), ws.Z.begin() + M * cols, 0.0);
    auto A = [&] (int r, int c) -> double& { return ws.A[(size_t) (r * M + c)]; };
    auto Z = [&] (int r, int c) -> double& { return ws.Z[(size_t) (r * cols + c)]; };
    auto stampG = [&] (int a, int b, double g)
    {
        if (a >= 0) A (a, a) += g;
        if (b >= 0) A (b, b) += g;
        if (a >= 0 && b >= 0) { A (a, b) -= g; A (b, a) -= g; }
    };
    int row = N;
    for (int e = 0; e < nElems; ++e)
    {
        const auto& el = elems[(size_t) e];
        switch (el.kind)
        {
            case Kind::R: stampG (el.a, el.b, 1.0 / el.value); break;
            case Kind::C: stampG (el.a, el.b, 1.0e-12); break;      // open (tiny leak keeps it regular)
            case Kind::L: stampG (el.a, el.b, 1.0e6); break;        // short
            case Kind::V:
                if (el.a >= 0) { A (el.a, row) += 1.0; A (row, el.a) += 1.0; }
                if (el.b >= 0) { A (el.b, row) -= 1.0; A (row, el.b) -= 1.0; }
                Z (row, el.idx) = 1.0;
                ++row; break;
            case Kind::E:
                if (el.a >= 0) { A (el.a, row) += 1.0; A (row, el.a) += 1.0; }
                if (el.b >= 0) { A (el.b, row) -= 1.0; A (row, el.b) -= 1.0; }
                if (el.c >= 0) A (row, el.c) -= el.value;
                if (el.d >= 0) A (row, el.d) += el.value;
                ++row; break;
            case Kind::G:
                if (el.a >= 0) { if (el.c >= 0) A (el.a, el.c) += el.value; if (el.d >= 0) A (el.a, el.d) -= el.value; }
                if (el.b >= 0) { if (el.c >= 0) A (el.b, el.c) -= el.value; if (el.d >= 0) A (el.b, el.d) += el.value; }
                break;
            case Kind::I:
                if (el.a >= 0) Z (el.a, el.idx) -= 1.0;
                if (el.b >= 0) Z (el.b, el.idx) += 1.0;
                break;
            case Kind::N:
                if (el.a >= 0) Z (el.a, nInputs + el.idx) -= 1.0;
                if (el.b >= 0) Z (el.b, nInputs + el.idx) += 1.0;
                break;
        }
    }
    for (int c = 0; c < M; ++c)
    {
        int p = c;
        double best = std::fabs (A (c, c));
        for (int r = c + 1; r < M; ++r)
            if (std::fabs (A (r, c)) > best) { best = std::fabs (A (r, c)); p = r; }
        if (best < 1.0e-300) return false;
        if (p != c)
        {
            for (int j = 0; j < M; ++j) std::swap (A (p, j), A (c, j));
            for (int j = 0; j < cols; ++j) std::swap (Z (p, j), Z (c, j));
        }
        const double inv = 1.0 / A (c, c);
        for (int r = c + 1; r < M; ++r)
        {
            const double f = A (r, c) * inv;
            if (f == 0.0) continue;
            for (int j = c; j < M; ++j) A (r, j) -= f * A (c, j);
            for (int j = 0; j < cols; ++j) Z (r, j) -= f * Z (c, j);
        }
    }
    for (int r = M - 1; r >= 0; --r)
    {
        const double inv = 1.0 / A (r, r);
        for (int j = 0; j < cols; ++j)
        {
            double acc = Z (r, j);
            for (int c = r + 1; c < M; ++c) acc -= A (r, c) * Z (c, j);
            Z (r, j) = acc * inv;
        }
    }
    auto nodeV = [&] (int node, int col) { return node >= 0 ? Z (node, col) : 0.0; };
    sm.nu = nInputs; sm.nnl = nNL; sm.nc = nCtrl; sm.no = nOut;
    for (int c = 0; c < nCtrl; ++c)
    {
        const auto& pr = ctrlNodes[(size_t) c];
        for (int q = 0; q < nInputs; ++q) sm.Ec[c][q] = nodeV (pr.p, q) - nodeV (pr.n, q);
        for (int m = 0; m < nNL; ++m)     sm.Fc[c][m] = nodeV (pr.p, nInputs + m) - nodeV (pr.n, nInputs + m);
    }
    for (int o = 0; o < nOut; ++o)
    {
        const auto& pr = outNodes[(size_t) o];
        for (int q = 0; q < nInputs; ++q) sm.Ou[o][q] = nodeV (pr.p, q) - nodeV (pr.n, q);
        for (int m = 0; m < nNL; ++m)     sm.Oi[o][m] = nodeV (pr.p, nInputs + m) - nodeV (pr.n, nInputs + m);
    }
    return true;
}

inline bool Circuit::steadyState (const double* u, const double* i)
{
    // (I − Φ) s = Γu + Λi
    double m[kMaxStates][kMaxStates + 1];
    for (int r = 0; r < ns; ++r)
    {
        for (int c = 0; c < ns; ++c)
            m[r][c] = (r == c ? 1.0 : 0.0) - Phi[(size_t) r][(size_t) c];
        double b = 0.0;
        for (int q = 0; q < nu; ++q) b += Gam[(size_t) r][(size_t) q] * u[q];
        for (int q = 0; q < nnl; ++q) b += Lam[(size_t) r][(size_t) q] * i[q];
        m[r][ns] = b;
    }
    for (int c = 0; c < ns; ++c)
    {
        int p = c;
        for (int r = c + 1; r < ns; ++r) if (std::fabs (m[r][c]) > std::fabs (m[p][c])) p = r;
        if (std::fabs (m[p][c]) < 1.0e-300) return false;
        if (p != c) for (int j = 0; j <= ns; ++j) std::swap (m[p][j], m[c][j]);
        for (int r = 0; r < ns; ++r)
        {
            if (r == c) continue;
            const double f = m[r][c] / m[c][c];
            if (f == 0.0) continue;
            for (int j = c; j <= ns; ++j) m[r][j] -= f * m[c][j];
        }
    }
    for (int r = 0; r < ns; ++r) { s[(size_t) r] = m[r][ns] / m[r][r]; sPrev[(size_t) r] = s[(size_t) r]; }
    return true;
}

inline std::complex<double> Circuit::digitalResponse (double f, int k, int o, const double* Jf) const
{
    using cd = std::complex<double>;
    // effective linear system with i = K·v, v = p + F i  →  i = (I − Jf F)⁻¹ Jf p
    double Kp[kMaxNL][kMaxCtrl] {};
    if (Jf != nullptr && nnl > 0)
    {
        double Mx[kMaxNL][kMaxNL];
        for (int a = 0; a < nnl; ++a)
            for (int b = 0; b < nnl; ++b)
            {
                double acc = (a == b ? 1.0 : 0.0);
                for (int c = 0; c < nc; ++c) acc -= Jf[a * nc + c] * Fc[(size_t) c][(size_t) b];
                Mx[a][b] = acc;
            }
        // invert (nnl ≤ 2)
        double Inv[kMaxNL][kMaxNL] {};
        if (nnl == 1) Inv[0][0] = 1.0 / Mx[0][0];
        else
        {
            const double det = Mx[0][0] * Mx[1][1] - Mx[0][1] * Mx[1][0];
            Inv[0][0] = Mx[1][1] / det; Inv[1][1] = Mx[0][0] / det;
            Inv[0][1] = -Mx[0][1] / det; Inv[1][0] = -Mx[1][0] / det;
        }
        for (int a = 0; a < nnl; ++a)
            for (int c = 0; c < nc; ++c)
            {
                double acc = 0.0;
                for (int b = 0; b < nnl; ++b) acc += Inv[a][b] * Jf[b * nc + c];
                Kp[a][c] = acc;
            }
    }
    // Φe = Φ + Λ Kp D ; Γe = Γk + Λ Kp Ek ; Oe = Oy + Oi Kp D ; Ue = Ouk + Oi Kp Ek
    double Pe[kMaxStates][kMaxStates], Ge[kMaxStates], Oe[kMaxStates];
    double LK[kMaxStates][kMaxCtrl] {}, OK[kMaxCtrl] {};
    for (int e = 0; e < ns; ++e)
        for (int c = 0; c < nc; ++c)
        {
            double acc = 0.0;
            for (int m = 0; m < nnl; ++m) acc += Lam[(size_t) e][(size_t) m] * Kp[m][c];
            LK[e][c] = acc;
        }
    for (int c = 0; c < nc; ++c)
    {
        double acc = 0.0;
        for (int m = 0; m < nnl; ++m) acc += Oi[(size_t) o][(size_t) m] * Kp[m][c];
        OK[c] = acc;
    }
    for (int e = 0; e < ns; ++e)
    {
        for (int j = 0; j < ns; ++j)
        {
            double acc = Phi[(size_t) e][(size_t) j];
            for (int c = 0; c < nc; ++c) acc += LK[e][c] * Dc[(size_t) c][(size_t) j];
            Pe[e][j] = acc;
        }
        double g = Gam[(size_t) e][(size_t) k];
        for (int c = 0; c < nc; ++c) g += LK[e][c] * Ec[(size_t) c][(size_t) k];
        Ge[e] = g;
    }
    for (int j = 0; j < ns; ++j)
    {
        double acc = Oy[(size_t) o][(size_t) j];
        for (int c = 0; c < nc; ++c) acc += OK[c] * Dc[(size_t) c][(size_t) j];
        Oe[j] = acc;
    }
    double Ue = Ou[(size_t) o][(size_t) k];
    for (int c = 0; c < nc; ++c) Ue += OK[c] * Ec[(size_t) c][(size_t) k];

    // H(z) = Ue + Oe (zI − Pe)⁻¹ Ge, with the timing y[n] = O s[n] + U u[n],
    // s[n+1] = P s[n] + G u[n]  →  s = (zI − P)⁻¹ G u
    const cd z = std::polar (1.0, 2.0 * kPi * f / fsCompiled);
    cd m[kMaxStates][kMaxStates + 1];
    for (int r = 0; r < ns; ++r)
    {
        for (int c = 0; c < ns; ++c) m[r][c] = (r == c ? z : cd (0.0)) - Pe[r][c];
        m[r][ns] = Ge[r];
    }
    for (int c = 0; c < ns; ++c)
    {
        int p = c;
        for (int r = c + 1; r < ns; ++r) if (std::abs (m[r][c]) > std::abs (m[p][c])) p = r;
        if (p != c) for (int j = 0; j <= ns; ++j) std::swap (m[p][j], m[c][j]);
        for (int r = 0; r < ns; ++r)
        {
            if (r == c) continue;
            const cd fct = m[r][c] / m[c][c];
            for (int j = c; j <= ns; ++j) m[r][j] -= fct * m[c][j];
        }
    }
    cd y = Ue;
    for (int r = 0; r < ns; ++r) y += Oe[r] * (m[r][ns] / m[r][r]);
    return y;
}

inline std::complex<double> Circuit::analogResponse (double f, int k, int o) const
{
    using cd = std::complex<double>;
    const int N = nNodes;
    int extra = 0;
    for (int e = 0; e < nElems; ++e)
        if (elems[(size_t) e].kind == Kind::V || elems[(size_t) e].kind == Kind::E) ++extra;
    const int M = N + extra;
    std::vector<cd> A ((size_t) (M * M), cd (0.0)), b ((size_t) M, cd (0.0));
    auto at = [&] (int r, int c) -> cd& { return A[(size_t) (r * M + c)]; };
    const cd sv (0.0, 2.0 * kPi * f);
    auto stamp = [&] (int a, int bb, cd y)
    {
        if (a >= 0) at (a, a) += y;
        if (bb >= 0) at (bb, bb) += y;
        if (a >= 0 && bb >= 0) { at (a, bb) -= y; at (bb, a) -= y; }
    };
    int row = N;
    for (int e = 0; e < nElems; ++e)
    {
        const auto& el = elems[(size_t) e];
        switch (el.kind)
        {
            case Kind::R: stamp (el.a, el.b, cd (1.0 / el.value)); break;
            case Kind::C: stamp (el.a, el.b, sv * el.value); break;
            case Kind::L: stamp (el.a, el.b, 1.0 / (sv * el.value)); break;
            case Kind::V:
                if (el.a >= 0) { at (el.a, row) += 1.0; at (row, el.a) += 1.0; }
                if (el.b >= 0) { at (el.b, row) -= 1.0; at (row, el.b) -= 1.0; }
                b[(size_t) row] = el.idx == k ? 1.0 : 0.0;
                ++row; break;
            case Kind::E:
                if (el.a >= 0) { at (el.a, row) += 1.0; at (row, el.a) += 1.0; }
                if (el.b >= 0) { at (el.b, row) -= 1.0; at (row, el.b) -= 1.0; }
                if (el.c >= 0) at (row, el.c) -= el.value;
                if (el.d >= 0) at (row, el.d) += el.value;
                ++row; break;
            case Kind::G:
                if (el.a >= 0) { if (el.c >= 0) at (el.a, el.c) += el.value; if (el.d >= 0) at (el.a, el.d) -= el.value; }
                if (el.b >= 0) { if (el.c >= 0) at (el.b, el.c) -= el.value; if (el.d >= 0) at (el.b, el.d) += el.value; }
                break;
            case Kind::I:
                if (el.idx == k) { if (el.a >= 0) b[(size_t) el.a] -= 1.0; if (el.b >= 0) b[(size_t) el.b] += 1.0; }
                break;
            case Kind::N: break;
        }
    }
    for (int c = 0; c < M; ++c)
    {
        int p = c;
        for (int r = c + 1; r < M; ++r) if (std::abs (at (r, c)) > std::abs (at (p, c))) p = r;
        if (std::abs (at (p, c)) < 1.0e-300) return cd (0.0);
        if (p != c) { for (int j = 0; j < M; ++j) std::swap (at (p, j), at (c, j)); std::swap (b[(size_t) p], b[(size_t) c]); }
        for (int r = c + 1; r < M; ++r)
        {
            const cd fct = at (r, c) / at (c, c);
            if (fct == cd (0.0)) continue;
            for (int j = c; j < M; ++j) at (r, j) -= fct * at (c, j);
            b[(size_t) r] -= fct * b[(size_t) c];
        }
    }
    std::vector<cd> x ((size_t) M);
    for (int r = M - 1; r >= 0; --r)
    {
        cd acc = b[(size_t) r];
        for (int j = r + 1; j < M; ++j) acc -= at (r, j) * x[(size_t) j];
        x[(size_t) r] = acc / at (r, r);
    }
    const auto& pr = outNodes[(size_t) o];
    return (pr.p >= 0 ? x[(size_t) pr.p] : cd (0.0)) - (pr.n >= 0 ? x[(size_t) pr.n] : cd (0.0));
}

} // namespace vt4
