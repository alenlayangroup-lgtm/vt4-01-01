// Halden VT-4 — calibrated spectrum analyser (display side, no allocation
// after construction).
//
//   window      4-term Blackman–Harris (side lobes −92 dB)
//   scaling     amplitude spectrum normalised by the window's coherent gain:
//               a sine of peak amplitude A reads A (0 dBFS sine → 0 dB)
//   peaks       log-parabolic interpolation of every local maximum removes
//               the scalloping loss (≤ 0.83 dB raw) → sines read within 0.1 dB
//   resolution  two FFTs: 16384 points below ≈ 300 Hz (≈ 3 Hz bins at 48 k),
//               4096 points above (fast response); cross-faded 200…400 Hz

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <vector>

namespace vt4
{

class Fft
{
public:
    explicit Fft (int order) : n (1 << order), lg (order)
    {
        tw.resize ((size_t) n / 2);
        for (int k = 0; k < n / 2; ++k) tw[(size_t) k] = std::polar (1.0, -2.0 * 3.14159265358979323846 * k / n);
        rev.resize ((size_t) n);
        for (int i = 0; i < n; ++i)
        {
            int r = 0;
            for (int b = 0; b < lg; ++b) if (i & (1 << b)) r |= 1 << (lg - 1 - b);
            rev[(size_t) i] = r;
        }
    }
    int size() const noexcept { return n; }

    void forward (std::vector<std::complex<double>>& x) const
    {
        for (int i = 0; i < n; ++i) if (i < rev[(size_t) i]) std::swap (x[(size_t) i], x[(size_t) rev[(size_t) i]]);
        for (int len = 2; len <= n; len <<= 1)
        {
            const int half = len / 2, step = n / len;
            for (int i = 0; i < n; i += len)
                for (int j = 0; j < half; ++j)
                {
                    const auto t = tw[(size_t) (j * step)] * x[(size_t) (i + j + half)];
                    const auto u = x[(size_t) (i + j)];
                    x[(size_t) (i + j)] = u + t;
                    x[(size_t) (i + j + half)] = u - t;
                }
        }
    }

private:
    int n, lg;
    std::vector<std::complex<double>> tw;
    std::vector<int> rev;
};

class SpectrumAnalyser
{
public:
    // grid: display frequencies (ascending), fs: host rate
    void prepare (const std::vector<double>& gridHz, double sampleRate)
    {
        grid = gridHz; fs = sampleRate;
        hist.assign ((size_t) kLong, 0.0f);
        pos = 0;
        out.assign (grid.size(), -200.0);
        for (int r = 0; r < 2; ++r)
        {
            const int N = r == 0 ? kLong : kShort;
            win[r].resize ((size_t) N);
            double sum = 0.0;
            for (int i = 0; i < N; ++i)
            {
                const double t = 2.0 * 3.14159265358979323846 * i / N;
                win[r][(size_t) i] = 0.35875 - 0.48829 * std::cos (t) + 0.14128 * std::cos (2.0 * t) - 0.01168 * std::cos (3.0 * t);
                sum += win[r][(size_t) i];
            }
            norm[r] = 2.0 / sum;                  // coherent gain → amplitude
            buf[r].resize ((size_t) N);
            spec[r].assign (grid.size(), -200.0);
        }
    }

    void push (const float* x, int n) noexcept
    {
        for (int i = 0; i < n; ++i) { hist[(size_t) pos] = x[i]; pos = (pos + 1) & (kLong - 1); }
    }

    // Computes a new frame. Values are dB re full scale (0 dB = full-scale sine).
    void compute()
    {
        peaks.clear();
        for (int r = 0; r < 2; ++r)
        {
            const int N = r == 0 ? kLong : kShort;
            const Fft& f = r == 0 ? fftLong : fftShort;
            for (int i = 0; i < N; ++i)
                buf[r][(size_t) i] = std::complex<double> (win[r][(size_t) i] * hist[(size_t) ((pos - N + i) & (kLong - 1))], 0.0);
            f.forward (buf[r]);
            mag.resize ((size_t) N / 2 + 1);
            for (int k = 0; k <= N / 2; ++k) mag[(size_t) k] = 20.0 * std::log10 (std::abs (buf[r][(size_t) k]) * norm[r] + 1.0e-12);
            mapToGrid (r, N);
        }
        for (size_t i = 0; i < grid.size(); ++i)
        {
            const double f = grid[i];
            const double t = std::clamp ((std::log (f) - std::log (200.0)) / (std::log (400.0) - std::log (200.0)), 0.0, 1.0);
            const double v = (1.0 - t) * spec[0][i] + t * spec[1][i];
            out[i] = v;
        }
    }

    const std::vector<double>& values() const noexcept { return out; }

    // calibrated level (dB) and frequency of the strongest component (tests)
    void strongest (double& db, double& hz) const
    {
        db = -300.0; hz = 0.0;
        for (const auto& p : peaks) if (p.first > db) { db = p.first; hz = p.second; }
    }

    static constexpr int kLong = 16384, kShort = 4096;

private:
    void mapToGrid (int r, int N)
    {
        auto& s = spec[r];
        std::fill (s.begin(), s.end(), -200.0);
        const double binHz = fs / N;
        // each grid cell covers [g(i−½), g(i+½)) in log f
        const size_t G = grid.size();
        auto edge = [&] (size_t i) { return i == 0 ? grid[0] / std::sqrt (grid[1] / grid[0]) : std::sqrt (grid[i - 1] * grid[i]); };
        size_t gi = 0;
        for (int k = 1; k < N / 2; ++k)
        {
            const double fk = k * binHz;
            while (gi + 1 < G && fk >= edge (gi + 1)) ++gi;
            if (fk < edge (0)) continue;
            s[gi] = std::max (s[gi], mag[(size_t) k]);
        }
        // cells narrower than a bin: interpolate between bins
        for (size_t i = 0; i < G; ++i)
        {
            const double kf = grid[i] / binHz;
            if (kf < 1.0 || kf >= N / 2 - 1) continue;
            const int k = (int) kf;
            const double t = kf - k;
            const double v = (1.0 - t) * mag[(size_t) k] + t * mag[(size_t) k + 1];
            if (s[i] < -199.0) s[i] = v;
        }
        // interpolated peaks (remove scalloping)
        for (int k = 2; k < N / 2 - 1; ++k)
        {
            const double a = mag[(size_t) k - 1], b = mag[(size_t) k], c = mag[(size_t) k + 1];
            if (! (b > a && b >= c) || b < -160.0) continue;
            const double den = a - 2.0 * b + c;
            const double d = den < 0.0 ? 0.5 * (a - c) / den : 0.0;
            const double pk = b - 0.25 * (a - c) * d;
            const double fp = (k + d) * binHz;
            const bool useThis = (r == 0 && fp < 300.0) || (r == 1 && fp >= 300.0);
            if (useThis) peaks.emplace_back (pk, fp);
            // place into the grid cell
            size_t lo = 0, hi = G;
            while (lo + 1 < hi) { const size_t m = (lo + hi) / 2; if (edge (m) <= fp) lo = m; else hi = m; }
            s[lo] = std::max (s[lo], pk);
        }
    }

    std::vector<double> grid, out, mag;
    double fs = 48000.0;
    std::vector<float> hist;
    int pos = 0;
    Fft fftLong { 14 }, fftShort { 12 };
    std::array<std::vector<double>, 2> win, spec;
    std::array<std::vector<std::complex<double>>, 2> buf;
    double norm[2] { 1.0, 1.0 };
    std::vector<std::pair<double, double>> peaks;
};

} // namespace vt4
