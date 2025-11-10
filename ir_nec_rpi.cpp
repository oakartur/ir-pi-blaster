// - TX: envia NECx (addr16=0x0707, cmd=0x02), portadora 38 kHz, duty ~33%
// - RX: decodifica NEC/NECx de um TSOP (ativo-baixo), imprime raw/addr/cmd
// Build: g++ -std=gnu++17 ir_nec_rpi.cpp -ldl -lpthread -O2 -o ir_nec_rpi
// Run:   sudo ./ir_nec_rpi --mode=rx|tx|all [--payload=0xFD020707]

#include <dlfcn.h>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <vector>
#include <thread>
#include <atomic>
#include <string>
#include <cctype>
#include <chrono>
#include <algorithm>
#include <mutex>
#include <condition_variable>
#include <fstream>
#include <sstream>
#include <optional>
#include <regex>
#include <array>

// ===================== GPIO backend selection =====================

namespace {

constexpr const char* kEnvModelOverride = "IR_PI_MODEL_OVERRIDE";
constexpr const char* kEnvForceLib      = "IR_PI_FORCE_LIB";

using gpioAlertFunc_t    = void (*)(int, int, uint32_t);
using gpioTimerFuncEx_t  = void (*)(void*);

struct gpioPulse_t {
    uint32_t gpioOn;
    uint32_t gpioOff;
    uint32_t usDelay;
};

struct BackendApi {
    void* handle = nullptr;
    std::string resolved_name;

    // --- Funções pigpio ---
    int       (*gpioInitialise)(void) = nullptr;
    void      (*gpioTerminate)(void)  = nullptr;
    int       (*gpioSetMode)(unsigned, unsigned) = nullptr;
    int       (*gpioWrite)(unsigned, unsigned) = nullptr;
    uint32_t  (*gpioTick)(void) = nullptr;
    void      (*gpioDelay)(unsigned) = nullptr;
    int       (*gpioWaveAddNew)(void) = nullptr;
    int       (*gpioWaveAddGeneric)(unsigned, gpioPulse_t*) = nullptr;
    int       (*gpioWaveCreate)(void) = nullptr;
    int       (*gpioWaveTxSend)(unsigned, unsigned) = nullptr;
    int       (*gpioWaveTxBusy)(void) = nullptr;
    int       (*gpioWaveDelete)(unsigned) = nullptr;
    int       (*gpioSetPullUpDown)(unsigned, unsigned) = nullptr;
    int       (*gpioSetAlertFunc)(unsigned, gpioAlertFunc_t) = nullptr;
    int       (*gpioSetTimerFuncEx)(unsigned, unsigned, gpioTimerFuncEx_t, void*) = nullptr;
    int       (*gpioGlitchFilter)(unsigned, unsigned) = nullptr;

    // --- Funções lgpio ---
    int  (*lgGpiochipOpen)(unsigned) = nullptr;
    int  (*lgGpiochipClose)(int) = nullptr;
    int  (*lgGpioClaimOutput)(int, unsigned, unsigned, unsigned) = nullptr;
    int  (*lgGpioWrite)(int, unsigned, unsigned) = nullptr;

    // estado interno
    bool is_lgpio = false;
    int chip_handle = -1;
};

static BackendApi g_backend;
static bool g_backend_loaded = false;
static std::string g_backend_error;

// Forward declarations for backend helpers used before their definitions.
static void unload_backend();

template <typename T>
static bool load_symbol(void* handle, T& fn, const char* name, std::string& err);

template <typename T>
static bool load_symbol_with_candidates(
    void* handle, T& fn, const std::vector<std::string>& names, std::string& err);

// ---- helpers ----
static inline std::string to_lower(const std::string& s) {
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    return out;
}

static std::optional<std::string> detect_pi_model() {
    const char* override = std::getenv(kEnvModelOverride);
    if (override && *override) return std::string(override);

    const std::array<const char*, 2> paths = {
        "/sys/firmware/devicetree/base/model",
        "/proc/device-tree/model"
    };
    for (const auto& path : paths) {
        std::ifstream f(path, std::ios::binary);
        if (!f.good()) continue;
        std::ostringstream ss;
        ss << f.rdbuf();
        std::string model = ss.str();
        model.erase(std::remove(model.begin(), model.end(), '\0'), model.end());
        if (!model.empty()) return model;
    }
    return std::nullopt;
}

static std::optional<int> parse_pi_version(const std::optional<std::string>& model) {
    if (!model) return std::nullopt;
    std::regex re(R"(Raspberry Pi\s*(?:Model\s*)?(\d+))", std::regex::icase);
    std::smatch m;
    if (std::regex_search(*model, m, re)) {
        try { return std::stoi(m[1].str()); }
        catch (...) { return std::nullopt; }
    }
    return std::nullopt;
}

// ===================== GPIO backend loading (robusto) =====================

bool load_backend_library(const std::string& lib_name_input)
{
    unload_backend();
    g_backend_error.clear();

    std::string lib_name = to_lower(lib_name_input);

    std::vector<std::string> candidates;

    if (lib_name.find('/') != std::string::npos) {
        candidates.push_back(lib_name);
    } else {
        candidates.push_back("lib" + lib_name + ".so");
        candidates.push_back("lib" + lib_name + ".so.0");
        candidates.push_back("lib" + lib_name + ".so.1");
        candidates.push_back(lib_name + ".so");
        candidates.push_back(lib_name);

        std::vector<std::string> search_dirs = {
            "/usr/lib/aarch64-linux-gnu/",
            "/lib/aarch64-linux-gnu/",
            "/usr/lib/arm-linux-gnueabihf/",
            "/lib/arm-linux-gnueabihf/",
            "/usr/local/lib/",
            "/usr/lib/",
            "/lib/"
        };

        for (const auto& dir : search_dirs) {
            candidates.push_back(dir + "lib" + lib_name + ".so");
            candidates.push_back(dir + "lib" + lib_name + ".so.0");
            candidates.push_back(dir + "lib" + lib_name + ".so.1");
        }
    }

    const char* last_error = nullptr;
    for (const auto& candidate : candidates) {
        dlerror();
        g_backend.handle = dlopen(candidate.c_str(), RTLD_LAZY | RTLD_LOCAL);
        if (g_backend.handle) {
            g_backend.resolved_name = candidate;
            std::fprintf(stderr, "[GPIO] Biblioteca carregada: %s\n", candidate.c_str());
            last_error = nullptr;
            break;
        } else {
            last_error = dlerror();
        }
    }

    if (!g_backend.handle) {
        g_backend_error = "Falha ao carregar biblioteca GPIO '" + lib_name + "'";
        if (last_error) {
            g_backend_error += ": ";
            g_backend_error += last_error;
        }
        return false;
    }

    const bool treat_as_pgpio =
        lib_name.find("pgpio") != std::string::npos ||
        g_backend.resolved_name.find("pgpio") != std::string::npos;

    auto symbol_candidates = [&](const char* base) {
        std::vector<std::string> names;
        names.emplace_back(base);
        if (treat_as_pgpio) {
            std::string base_str(base);
            names.emplace_back("p" + base_str);
            if (base_str.rfind("gpio", 0) == 0) {
                std::string suffix = base_str.substr(4);
                names.emplace_back("pg" + suffix);
                names.emplace_back("pgpio" + suffix);
            }
        }
        std::sort(names.begin(), names.end());
        names.erase(std::unique(names.begin(), names.end()), names.end());
        return names;
    };

    auto require = [&](auto& fn, const char* symbol) {
        auto candidates = symbol_candidates(symbol);
        if (!load_symbol_with_candidates(g_backend.handle, fn, candidates, g_backend_error)) {
            std::ostringstream oss;
            oss << "[GPIO] Falha ao resolver símbolo " << symbol << " em "
                << g_backend.resolved_name;
            if (!candidates.empty()) {
                oss << " (tentativas:";
                for (const auto& name : candidates) oss << ' ' << name;
                oss << ')';
            }
            std::fprintf(stderr, "%s\n", oss.str().c_str());
            unload_backend();
            return false;
        }
        return true;
    };

    bool loaded_any = false;

    // tenta pigpio primeiro
    if (load_symbol(g_backend.handle, g_backend.gpioInitialise, "gpioInitialise", g_backend_error)) {
        if (!require(g_backend.gpioTerminate,      "gpioTerminate"))      return false;
        if (!require(g_backend.gpioSetMode,        "gpioSetMode"))        return false;
        if (!require(g_backend.gpioWrite,          "gpioWrite"))          return false;
        if (!require(g_backend.gpioTick,           "gpioTick"))           return false;
        if (!require(g_backend.gpioDelay,          "gpioDelay"))          return false;
        if (!require(g_backend.gpioWaveAddNew,     "gpioWaveAddNew"))     return false;
        if (!require(g_backend.gpioWaveAddGeneric, "gpioWaveAddGeneric")) return false;
        if (!require(g_backend.gpioWaveCreate,     "gpioWaveCreate"))     return false;
        if (!require(g_backend.gpioWaveTxSend,     "gpioWaveTxSend"))     return false;
        if (!require(g_backend.gpioWaveTxBusy,     "gpioWaveTxBusy"))     return false;
        if (!require(g_backend.gpioWaveDelete,     "gpioWaveDelete"))     return false;
        if (!require(g_backend.gpioSetPullUpDown,  "gpioSetPullUpDown"))  return false;
        if (!require(g_backend.gpioSetAlertFunc,   "gpioSetAlertFunc"))   return false;
        if (!require(g_backend.gpioSetTimerFuncEx, "gpioSetTimerFuncEx")) return false;
        if (!require(g_backend.gpioGlitchFilter,   "gpioGlitchFilter"))   return false;
        g_backend.is_lgpio = false;
        loaded_any = true;
    }
    // se não era pigpio, tenta API lgpio (OBS: não há equivalentes aos wave/alert pigpio)
    else if (load_symbol(g_backend.handle, g_backend.lgGpiochipOpen, "lgGpiochipOpen", g_backend_error)) {
        require(g_backend.lgGpiochipClose, "lgGpiochipClose");
        require(g_backend.lgGpioClaimOutput, "lgGpioClaimOutput");
        require(g_backend.lgGpioWrite, "lgGpioWrite");
        g_backend.is_lgpio = true;
        loaded_any = true;
    }

    if (!loaded_any) {
        g_backend_error = "Nenhuma API GPIO compatível encontrada em " + g_backend.resolved_name;
        unload_backend();
        return false;
    }

    g_backend_loaded = true;
    g_backend_error.clear();
    return true;
}


// --- Abstrações unificadas ---

static bool gpio_backend_init()
{
    if (g_backend.is_lgpio) {
        g_backend.chip_handle = g_backend.lgGpiochipOpen(0);
        return (g_backend.chip_handle >= 0);
    } else {
        return g_backend.gpioInitialise && (g_backend.gpioInitialise() >= 0);
    }
}

static void gpio_backend_close()
{
    if (g_backend.is_lgpio) {
        if (g_backend.chip_handle >= 0) g_backend.lgGpiochipClose(g_backend.chip_handle);
        g_backend.chip_handle = -1;
    } else if (g_backend.gpioTerminate) {
        g_backend.gpioTerminate();
    }
}

static bool gpio_backend_write(unsigned pin, unsigned value)
{
    if (g_backend.is_lgpio) {
        if (g_backend.chip_handle < 0) return false;
        return g_backend.lgGpioWrite(g_backend.chip_handle, pin, value) == 0;
    } else {
        return g_backend.gpioWrite && (g_backend.gpioWrite(pin, value) == 0);
    }
}

std::string select_gpio_library()
{
    const char* forced = std::getenv(kEnvForceLib);
    if (forced && std::strlen(forced) > 0)
        return to_lower(forced);

    return "pigpio";
}

// ===================== Backend loader utils =====================

static void unload_backend()
{
    if (g_backend.handle) {
        dlclose(g_backend.handle);
        g_backend.handle = nullptr;
    }
    g_backend_loaded = false;
}

template<typename T>
static bool load_symbol(void* handle, T& fn, const char* name, std::string& err)
{
    fn = reinterpret_cast<T>(dlsym(handle, name));
    if (!fn) {
        const char* e = dlerror();
        err = std::string("Símbolo ausente: ") + name +
              (e ? std::string(" (") + e + ")" : "");
        return false;
    }
    return true;
}

template<typename T>
static bool load_symbol_with_candidates(
    void* handle, T& fn, const std::vector<std::string>& names, std::string& err)
{
    std::string last_error;
    for (const auto& name : names) {
        dlerror();
        fn = reinterpret_cast<T>(dlsym(handle, name.c_str()));
        if (fn) {
            err.clear();
            return true;
        }
        if (const char* e = dlerror()) {
            last_error.assign(e);
        } else {
            last_error.clear();
        }
    }

    std::ostringstream oss;
    oss << "Símbolo ausente:";
    for (const auto& name : names) {
        oss << ' ' << name;
    }
    if (!last_error.empty()) {
        oss << " (" << last_error << ")";
    }
    err = oss.str();
    return false;
}

static bool ensure_backend_loaded()
{
    if (g_backend_loaded)
        return true;

    std::string selected = select_gpio_library();

    std::vector<std::string> tries;
    if (const char* env = std::getenv("IR_PI_FORCE_LIB"); env && *env)
        tries.push_back(env);
    tries.push_back(selected);
    if (selected != "pigpio")
        tries.push_back("pigpio");

    for (const auto& lib : tries) {
        std::fprintf(stderr, "[GPIO] Tentando carregar backend: %s\n", lib.c_str());
        if (load_backend_library(lib)) {
            std::fprintf(stderr, "[GPIO] Backend ativo: %s\n", g_backend.resolved_name.c_str());
            return true;
        } else {
            std::fprintf(stderr, "[GPIO] Falha: %s\n", g_backend_error.c_str());
        }
    }

    return false;
}

} //namespace


#define PI_INPUT   0
#define PI_OUTPUT  1
#define PI_PUD_OFF 0
#define PI_PUD_DOWN 1
#define PI_PUD_UP  2
#define PI_WAVE_MODE_ONE_SHOT 0u

#define gpioInitialise()          (g_backend.gpioInitialise())
#define gpioTerminate()           (g_backend.gpioTerminate())
#define gpioSetMode(gpio, mode)   (g_backend.gpioSetMode((gpio), (mode)))
#define gpioWrite(gpio, level)    (g_backend.gpioWrite((gpio), (level)))
#define gpioTick()                (g_backend.gpioTick())
#define gpioDelay(us)             (g_backend.gpioDelay((us)))
#define gpioWaveAddNew()          (g_backend.gpioWaveAddNew())
#define gpioWaveAddGeneric(n, p)  (g_backend.gpioWaveAddGeneric((n), (p)))
#define gpioWaveCreate()          (g_backend.gpioWaveCreate())
#define gpioWaveTxSend(w, m)      (g_backend.gpioWaveTxSend((w), (m)))
#define gpioWaveTxBusy()          (g_backend.gpioWaveTxBusy())
#define gpioWaveDelete(w)         (g_backend.gpioWaveDelete((w)))
#define gpioSetPullUpDown(g, p)   (g_backend.gpioSetPullUpDown((g), (p)))
#define gpioSetAlertFunc(g, cb)   (g_backend.gpioSetAlertFunc((g), (cb)))
#define gpioSetTimerFuncEx(t, i, cb, ud) \
    (g_backend.gpioSetTimerFuncEx((t), (i), (cb), (ud)))
#define gpioGlitchFilter(g, l)    (g_backend.gpioGlitchFilter((g), (l)))

// ===================== CONFIG GLOBAIS =====================

// Sugeridos: TX=18 (PWM0), RX=23. Você pode mudar.
static int GPIO_TX = 23;
static int GPIO_RX = 14;
static int GPIO_BTN = 17;

// Modo de execução
enum RunMode { MODE_TX_ONESHOT = 0, MODE_ALL, MODE_RX, MODE_TX };
static RunMode g_mode = MODE_TX_ONESHOT;

static std::string g_cmd_name;
static std::string g_map_path ="/home/aac/projects/ir-controller/button-map";

// Portadora e duty
static unsigned CARRIER_HZ    = 38000;  // 36k..40k se necessário
static float    DUTY_FRACTION = 0.33f;  // 0.25..0.5

// Payload NECx padrão
static uint16_t NECX_ADDR16 = 0x0707;
static uint8_t  NECX_CMD    = 0x02;

// Timings NEC (us)
static const unsigned NEC_MARK_US         = 560;
static const unsigned NEC_SPACE_0_US      = 560;
static const unsigned NEC_SPACE_1_US      = 1690;
static unsigned NEC_LEADER_MARK_US  = 4500;
static unsigned NEC_LEADER_SPACE_US = 4500;
static const unsigned NEC_REPEAT_LEAD_US  = 2250;   // repeat curto (pós-leader)
static const unsigned INTERFRAME_GAP_US   = 40000;  // gap pós-frame (TX)

// Heurística RX
static const uint32_t END_GAP_US          = 8000;   // fecha frame se 8 ms sem novas bordas

// ===================== Estruturas / Estado =====================

struct Pulse  { int level; uint32_t dur_us; };                  // nível + duração
struct Symbol { uint8_t level0; uint32_t dur0; uint8_t level1; uint32_t dur1; };

static std::atomic<bool> g_running{true};

static std::vector<Pulse> g_cur_pulses;
static std::mutex         g_mtx;

// Para fechamento por inatividade (um único pipeline de RX)
static std::atomic<uint32_t> g_last_tick_atomic{0};  // último tick visto pelo alert_cb
static uint32_t              g_last_tick = 0;        // último tick “commitado” no buffer
static int                   g_last_level = 1;       // idle alto (TSOP ativo-baixo)

// Sinaliza TX por botao
static std::mutex		g_tx_mtx;
static std::condition_variable	g_tx_cv;
static bool			g_tx_request = false;
static bool			g_btn_pressed = false;

static bool parse_hex_u32(const std::string& s, uint32_t& out);
static int  build_wave_full(unsigned gpio_tx, uint16_t addr, uint8_t cmd);
static void on_sigint(int);
static int  tx_once_and_exit(); // se for usada no main

// ===================== Helpers genéricos =====================

static inline bool in_range_u32(uint32_t v, uint32_t lo, uint32_t hi) {
    return (v >= lo) && (v <= hi);
}
static inline bool is_approx_1T(uint32_t v, float T) {
    return (v >= (uint32_t)(0.55f*T) && v <= (uint32_t)(1.60f*T));
}
static inline bool is_approx_3T(uint32_t v, float T) {
    return (v >= (uint32_t)(2.0f*T)  && v <= (uint32_t)(4.2f*T));
}

// ===================== CLI helpers =====================
// (adicione uma enum/flag simples)
enum LeaderMode { LEADER_FULL, LEADER_HALF };
static LeaderMode g_leader_mode = LEADER_HALF; // <== default half para seguir seu controle

// ===================== ler button-map =====================
static bool load_map_payload(const std::string& path, const std::string& key, uint32_t& out_le){
    std::ifstream f(path);
    if (!f.good()) return false;
    std::string line;
    while (std::getline(f, line)) {
        // remove comentarios
        auto hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        // trim simples
        auto trim = [](std::string &s){
            const char* ws = " \t\r\n";
            s.erase(0, s.find_first_not_of(ws));
            s.erase(s.find_last_not_of(ws)+1);
        };
        trim(line);
        if (line.empty()) continue;

        auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string k = line.substr(0, eq);
        std::string v = line.substr(eq+1);
        trim(k); trim(v);

        if (k == key) {
            uint32_t le = 0;
            if (!parse_hex_u32(v, le)) return false;
            out_le = le;
            return true;
        }
    }
    return false;
}
// ===================== TX once =====================
static int tx_once_and_exit(){
    gpioSetMode(GPIO_TX, PI_OUTPUT);
    gpioWrite(GPIO_TX, 0);

    int wid_full = build_wave_full(GPIO_TX, NECX_ADDR16, NECX_CMD);
    if (wid_full < 0) {
        std::fprintf(stderr, "Falha build wave full\n");
        return 1;
    }
    uint32_t t0 = gpioTick();
    gpioWaveTxSend(wid_full, PI_WAVE_MODE_ONE_SHOT);
    while (gpioWaveTxBusy()) gpioDelay(500);
    uint32_t t1 = gpioTick();
    std::printf("TX ONE-SHOT done (delta=%u us)\n", (t1 - t0));
    gpioWaveDelete(wid_full);
    return 0;
}

// ===================== Pulses -> Symbols =====================

static void pulses_to_symbols(const std::vector<Pulse>& p, std::vector<Symbol>& out) {
    out.clear();
    if (p.size() < 2) return;
    for (size_t i = 0; i + 1 < p.size(); i += 2) {
        Symbol s{};
        s.level0 = (uint8_t)p[i].level;   s.dur0 = p[i].dur_us;
        s.level1 = (uint8_t)p[i+1].level; s.dur1 = p[i+1].dur_us;
        out.push_back(s);
    }
}

// ===================== Líder, Repeat e Bits =====================

static bool find_leader_ratio(const std::vector<Symbol>& sym, size_t &idx, uint16_t &mark_level,
                              uint32_t &m_us, uint32_t &s_us)
{
    for (size_t i = 0; i < sym.size(); ++i) {
        const auto &p = sym[i];
        uint32_t m0 = p.dur0, sp0 = p.dur1; // (mark, space)
        uint32_t sp1 = p.dur0, m1  = p.dur1; // (space, mark) — caso invertido

        bool case0_full = in_range_u32(m0, 6000, 12000) && in_range_u32(sp0, 2500, 7000);
        bool case1_full = in_range_u32(m1, 6000, 12000) && in_range_u32(sp1, 2500, 7000);
        bool case0_half = in_range_u32(m0, 4000,  6500) && in_range_u32(sp0, 2500, 7000);
        bool case1_half = in_range_u32(m1, 4000,  6500) && in_range_u32(sp1, 2500, 7000);
        bool case0_rep  = in_range_u32(m0, 6000, 12000) && in_range_u32(sp0, 1500, 3000);
        bool case1_rep  = in_range_u32(m1, 6000, 12000) && in_range_u32(sp1, 1500, 3000);

        if (case0_full || case0_half || case0_rep) { idx = i; mark_level = p.level0; m_us = m0; s_us = sp0; return true; }
        if (case1_full || case1_half || case1_rep) { idx = i; mark_level = p.level1; m_us = m1; s_us = sp1; return true; }
    }
    return false;
}

static bool is_nec_repeat_ratio(const std::vector<Symbol>& sym, size_t i0, uint16_t mark_level, float T) {
    if (i0 >= sym.size()) return false;
    const auto &s0 = sym[i0];
    uint32_t m0 = (s0.level0 == mark_level) ? s0.dur0 : s0.dur1;
    uint32_t sp = (s0.level0 == mark_level) ? s0.dur1 : s0.dur0;
    if (!(in_range_u32(m0, (uint32_t)(10.0f*T), (uint32_t)(28.0f*T)) &&
          in_range_u32(sp, (uint32_t)( 2.0f*T), (uint32_t)( 7.0f*T)))) return false;

    if (i0 + 1 >= sym.size()) return false;
    const auto &s1 = sym[i0+1];
    uint32_t m1 = (s1.level0 == mark_level) ? s1.dur0 : s1.dur1;
    return is_approx_1T(m1, T); // 560 us approx
}

static bool nec_bits_decode_ratio(const std::vector<Symbol>& sym, uint32_t &out32) {
    size_t i0 = 0; uint16_t mark_level = 1; uint32_t m_us = 0, s_us = 0;
    if (!find_leader_ratio(sym, i0, mark_level, m_us, s_us)) return false;
    float Tm = (m_us > 7000) ? (m_us / 16.0f) : (m_us / 8.0f);
    float Ts = s_us /  8.0f;
    float T  = (Tm + Ts) * 0.5f;

    uint32_t data = 0;
    size_t bitStart = i0 + 1;
    if (bitStart + 32 > sym.size()) return false;

    for (int i = 0; i < 32; ++i) {
        const auto &b = sym[bitStart + i];
        uint32_t d_mark  = (b.level0 == mark_level) ? b.dur0 : b.dur1;
        uint32_t d_space = (b.level0 == mark_level) ? b.dur1 : b.dur0;
        if (!is_approx_1T(d_mark, T)) return false;
        if      (is_approx_3T(d_space, T)) data |= (1u << i); // ONE
        else if (is_approx_1T(d_space, T)) {/* ZERO */}
        else return false;
    }
    out32 = data;
    return true;
}

// ===================== Interpretação NEC/NECx =====================

enum nec_mode_t { NEC_MODE_NEC = 0, NEC_MODE_NECX = 1, NEC_MODE_AUTO = 2 };
static nec_mode_t g_nec_mode = NEC_MODE_AUTO;

static bool interpret_code(uint32_t raw, nec_mode_t mode, uint32_t *addr_out, uint32_t *cmd_out, const char **which) {
    uint8_t b0 = (raw >>  0) & 0xFF;
    uint8_t b1 = (raw >>  8) & 0xFF;
    uint8_t b2 = (raw >> 16) & 0xFF;
    uint8_t b3 = (raw >> 24) & 0xFF;

    bool nec_ok  = ((uint8_t)~b0 == b1) && ((uint8_t)~b2 == b3);
    bool necx_ok = ((uint8_t)~b2 == b3) && ((uint8_t)~b0 != b1);

    if (mode == NEC_MODE_NEC) {
        if (!nec_ok) return false;
        *addr_out = b0; *cmd_out = b2; *which = "NEC";  return true;
    } else if (mode == NEC_MODE_NECX) {
        if (!necx_ok) return false;
        *addr_out = (uint32_t)b0 | ((uint32_t)b1 << 8); *cmd_out = b2; *which = "NECx"; return true;
    } else {
        if (nec_ok)  { *addr_out = b0; *cmd_out = b2; *which = "NEC";  return true; }
        if (necx_ok) { *addr_out = (uint32_t)b0 | ((uint32_t)b1 << 8); *cmd_out = b2; *which = "NECx"; return true; }
        return false;
    }
}

// ===================== TX (pigpio wave) =====================

static void wave_append_carrier(std::vector<gpioPulse_t>& pulses, unsigned gpio, unsigned mark_us)
{
    const unsigned period_us = (unsigned)(1'000'000.0 / CARRIER_HZ + 0.5);        // ~26 us @38k
    const unsigned on_us  = (unsigned)(period_us * DUTY_FRACTION + 0.5);           // ~9 us
    const unsigned off_us = (period_us > on_us) ? (period_us - on_us) : 1;

    unsigned remaining = mark_us;
    uint32_t maskOn  = (1u << gpio);
    uint32_t maskOff = (1u << gpio);

    while (remaining > period_us) {
        pulses.push_back(gpioPulse_t{ maskOn, 0,       on_us  });
        pulses.push_back(gpioPulse_t{ 0,      maskOff, off_us });
        remaining -= period_us;
    }
    if (remaining > 1) {
        unsigned on_last  = std::min(on_us, remaining);
        unsigned off_last = (remaining > on_last) ? (remaining - on_last) : 0;
        pulses.push_back(gpioPulse_t{ maskOn, 0,       on_last });
        if (off_last) pulses.push_back(gpioPulse_t{ 0,  maskOff, off_last });
    }
}

static void wave_append_space(std::vector<gpioPulse_t>& pulses, unsigned gpio, unsigned space_us)
{
    // mantém LOW por space_us
    uint32_t maskOff = (1u << gpio);
    pulses.push_back(gpioPulse_t{ 0, maskOff, space_us });
}

static int build_wave_full(unsigned gpio_tx, uint16_t addr, uint8_t cmd)
{
    std::vector<gpioPulse_t> pulses;
    pulses.reserve(4000);

    // Leader
    wave_append_carrier(pulses, gpio_tx, NEC_LEADER_MARK_US);
    wave_append_space  (pulses, gpio_tx, NEC_LEADER_SPACE_US);

    auto emit_byte_lsbf = [&](uint8_t b) {
        for (int i = 0; i < 8; ++i) {
            wave_append_carrier(pulses, gpio_tx, NEC_MARK_US);
            wave_append_space  (pulses, gpio_tx, ((b >> i) & 0x1) ? NEC_SPACE_1_US : NEC_SPACE_0_US);
        }
    };

    emit_byte_lsbf((uint8_t)(addr & 0xFF));
    emit_byte_lsbf((uint8_t)((addr >> 8) & 0xFF));
    emit_byte_lsbf(cmd);
    emit_byte_lsbf((uint8_t)~cmd);

    // Trailer + idle
    wave_append_carrier(pulses, gpio_tx, NEC_MARK_US);
    wave_append_space  (pulses, gpio_tx, INTERFRAME_GAP_US);

    gpioWaveAddNew();
    gpioWaveAddGeneric(pulses.size(), pulses.data());
    int wid = gpioWaveCreate();
    return wid; // lembrar de gpioWaveDelete(wid)
}

static int build_wave_repeat(unsigned gpio_tx)
{
    std::vector<gpioPulse_t> pulses;
    pulses.reserve(1000);

    wave_append_carrier(pulses, gpio_tx, NEC_LEADER_MARK_US);
    wave_append_space  (pulses, gpio_tx, NEC_REPEAT_LEAD_US);
    wave_append_carrier(pulses, gpio_tx, NEC_MARK_US);
    wave_append_space  (pulses, gpio_tx, INTERFRAME_GAP_US);

    gpioWaveAddNew();
    gpioWaveAddGeneric(pulses.size(), pulses.data());
    int wid = gpioWaveCreate();
    return wid;
}

static void tx_thread()
{
    gpioSetMode(GPIO_TX, PI_OUTPUT);
    gpioWrite(GPIO_TX, 0);

    int wid_full   = build_wave_full(GPIO_TX, NECX_ADDR16, NECX_CMD);
    if (wid_full < 0) { std::fprintf(stderr, "Falha build wave full\n"); return; }

    std::printf("TX pronto no gpio %d (carrier ~%u Hz, duty ~%.0f%%). Pressione o botao em GPIO %d.\n",GPIO_TX, CARRIER_HZ, DUTY_FRACTION*100.0f, GPIO_BTN);

    while (g_running.load()){
        std::unique_lock<std::mutex> lk(g_tx_mtx);
        g_tx_cv.wait(lk, []{ return g_tx_request || !g_running.load(); });
        if (!g_running.load()) break;
        g_tx_request = false;
        lk.unlock();

        uint32_t t0 = gpioTick();
        gpioWaveTxSend(wid_full, PI_WAVE_MODE_ONE_SHOT);
        while (gpioWaveTxBusy() && g_running.load()) gpioDelay(1000);
        uint32_t t1 = gpioTick();
        std::printf("TX FULL done @ %u us (delta=%u us)\n", t1, (t1 - t0));
    }
    gpioWaveDelete(wid_full);
}

// ===================== RX (um único pipeline + flush por inatividade) =====================

static void flush_frame_and_decode()
{
    // Converte pulses -> symbols
    std::vector<Symbol> syms;
    {
        std::lock_guard<std::mutex> lk(g_mtx);
        if (g_cur_pulses.size() < 4) { g_cur_pulses.clear(); return; }
        pulses_to_symbols(g_cur_pulses, syms);
        g_cur_pulses.clear();
    }

    // Tenta reconhecer REPEAT curto
    size_t i0 = 0; uint16_t mark_level = 1; uint32_t m_us=0, s_us=0;
    bool have_leader = find_leader_ratio(syms, i0, mark_level, m_us, s_us);
    float T = have_leader ? ( ((m_us > 7000) ? (m_us/16.0f) : (m_us/8.0f)) + (s_us/8.0f) ) * 0.5f : 560.0f;

    static uint32_t last_raw = 0; static bool have_last = false;
    if (have_leader && is_nec_repeat_ratio(syms, i0, mark_level, T)) {
        if (have_last) {
            std::printf("REPEAT 0x%08" PRIX32 "\n", last_raw);
        } else {
            std::printf("REPEAT sem código anterior\n");
        }
        return;
    }

    // Bits 32
    uint32_t raw = 0; bool ok = nec_bits_decode_ratio(syms, raw);
    if (!ok) {
        if (syms.size() <= 6) std::printf("quadro curto e nao decodificado (n=%zu)\n", syms.size());
        else                  std::printf("frame NAO decodificado\n");
        return;
    }

    uint32_t addr=0, cmd=0; const char* which="?";
    if (interpret_code(raw, g_nec_mode, &addr, &cmd, &which)) {
        std::printf("%s: 0x%08" PRIX32 "\n", which, raw);
        if (std::strcmp(which,"NEC")==0)
            std::printf("RX OK (%s) -> raw=0x%08" PRIX32 " addr=0x%02" PRIX32 " cmd=0x%02" PRIX32 "\n",
                        which, raw, addr & 0xFF, cmd & 0xFF);
        else
            std::printf("RX OK (%s) -> raw=0x%08" PRIX32 " addr16=0x%04" PRIX32 " cmd=0x%02" PRIX32 "\n",
                        which, raw, addr & 0xFFFF, cmd & 0xFF);
        last_raw = raw; have_last = true;
    } else {
        std::printf("bits OK mas bytes nao batem (modo=%d)\n", (int)g_nec_mode);
    }
}

// callback de borda (único)
static void alert_cb(int gpio, int level, uint32_t tick)
{
    (void)gpio;
    // acumula duração do nível anterior
    if (g_last_tick != 0) {
        uint32_t dur = tick - g_last_tick; // pigpio lida com wrap de uint32
        std::lock_guard<std::mutex> lk(g_mtx);
        g_cur_pulses.push_back(Pulse{ g_last_level, dur });
    }
    g_last_level = level;
    g_last_tick  = tick;

    // marca timestamp para o timer de inatividade
    g_last_tick_atomic.store(tick, std::memory_order_relaxed);
}

// timer de 1 ms: fecha se END_GAP_US sem bordas
static void gap_timer_cb(void* /*userdata*/)
{
    uint32_t last = g_last_tick_atomic.load(std::memory_order_relaxed);
    if (last == 0) return;
    uint32_t now = gpioTick();
    uint32_t idle = now - last;
    if (idle >= END_GAP_US) {
        // fecha por inatividade
        flush_frame_and_decode();
        // limpa buffer para próximo frame
        std::lock_guard<std::mutex> lk(g_mtx);
        g_cur_pulses.clear();
        g_last_tick = 0; // evita flush repetido até a próxima borda iniciar novo frame
    }
}

static void rx_start()
{
    gpioSetMode(GPIO_RX, PI_INPUT);
    // Se o TSOP já tem pull-up externo, deixe OFF (recomendado). Use PI_PUD_UP se precisar.
    gpioSetPullUpDown(GPIO_RX, PI_PUD_OFF);

    // registra o ÚNICO callback
    gpioSetAlertFunc(GPIO_RX, alert_cb);

    // timer periódico 1 ms para fechamento por inatividade
    gpioSetTimerFuncEx(0 /*timer id*/, 10 /*ms*/, gap_timer_cb, nullptr);

    std::printf("Aguardando NEC/NECx no GPIO %d (modo=%d)\n", GPIO_RX, (int)g_nec_mode);
}

// ===================== CLI helpers =====================

static bool parse_hex_u32(const std::string& s, uint32_t& out) {
    std::string t = s;
    if (t.rfind("0x", 0) == 0 || t.rfind("0X", 0) == 0) t = t.substr(2);
    if (t.empty() || t.size() > 8) return false;
    for (char c : t) if (!std::isxdigit(static_cast<unsigned char>(c))) return false;
    out = static_cast<uint32_t>(std::strtoul(t.c_str(), nullptr, 16));
    return true;
}

static bool apply_payload_le(uint32_t le){
    uint8_t p0 =  le        & 0xFF; // addrL
    uint8_t p1 = (le >> 8)  & 0xFF; // addrH
    uint8_t p2 = (le >> 16) & 0xFF; // cmd
    uint8_t p3 = (le >> 24) & 0xFF; // ~cmd
    if ((uint8_t)~p2 != p3) {
        std::fprintf(stderr, "Aviso: payload 0x%08" PRIX32 " tem ~cmd inconsistente (cmd=0x%02X, ~cmd=0x%02X)\n", le, p2, p3);
    }
    NECX_ADDR16 = (uint16_t)p0 | ((uint16_t)p1 << 8);
    NECX_CMD    = p2;
    std::printf("Payload aplicado: LE=0x%08" PRIX32 " -> addr16=0x%04X cmd=0x%02X (~cmd=0x%02X)\n",
                le, NECX_ADDR16, NECX_CMD, (uint8_t)~NECX_CMD);
    return true;
}

static void on_sigint(int) {
    g_running.store(false);  // sinaliza para o loop principal encerrar
}

// ==================== Botao ====================
static void btn_cb(int gpio, int level, uint32_t /*tick*/){
    (void)gpio;

    if (level == 0){ // press
            g_btn_pressed = true;
	    {
                std::lock_guard<std::mutex> lk(g_tx_mtx);
                g_tx_request = true;
            }
            g_tx_cv.notify_one();
    } else if (level == 1){ // release
        g_btn_pressed = false;
    }
}

static void button_start(){
    gpioSetMode(GPIO_BTN, PI_INPUT);
    gpioSetPullUpDown(GPIO_BTN, PI_PUD_UP);
    gpioGlitchFilter(GPIO_BTN, 5000);
    gpioSetAlertFunc(GPIO_BTN, btn_cb);
    std::printf("Botao de TX em GPIO %d (pull-up interno, para GND). NUNCA use 5V no GPIO.\n", GPIO_BTN);
}


// ===================== MAIN =====================
int main(int argc, char** argv)
{
    uint32_t payload_le = 0;

    // Parse CLI
    for (int i=1; i<argc; ++i) {
        std::string arg = argv[i];
        auto eat = [&](const char* k)->std::string{
            std::string key(k);
            if (arg.rfind(key, 0) == 0) {
                auto pos = arg.find('=');
                if (pos != std::string::npos) return arg.substr(pos+1);
            }
            return {};
        };
        if (arg == "-m" || arg == "--mode") {
            if (i+1 >= argc) { std::fprintf(stderr, "Faltou valor para %s\n", arg.c_str()); return 1; }
            std::string v = argv[++i];
            for (auto& c: v) c = std::toupper(static_cast<unsigned char>(c));
            if      (v == "RX")  g_mode = MODE_RX;
            else if (v == "TX")  g_mode = MODE_TX;
            else if (v == "ALL") g_mode = MODE_ALL;
            else { std::fprintf(stderr, "Modo invalido: %s (use RX|TX|ALL)\n", v.c_str()); return 1; }
        } else if (arg.rfind("--mode=",0) == 0) {
            std::string v = eat("--mode=");
            for (auto& c: v) c = std::toupper(static_cast<unsigned char>(c));
            if      (v == "RX")  g_mode = MODE_RX;
            else if (v == "TX")  g_mode = MODE_TX;
            else if (v == "ALL") g_mode = MODE_ALL;
            else { std::fprintf(stderr, "Modo invalido: %s (use RX|TX|ALL)\n", v.c_str()); return 1; }
        } else if (arg == "-p" || arg == "--payload") {
            if (i+1 >= argc) { std::fprintf(stderr, "Faltou valor para %s\n", arg.c_str()); return 1; }
            std::string v = argv[++i];
            if (!parse_hex_u32(v, payload_le)) { std::fprintf(stderr, "Payload invalido: %s\n", v.c_str()); return 1; }
        } else if (arg.rfind("--payload=",0) == 0) {
            std::string v = eat("--payload=");
            if (!parse_hex_u32(v, payload_le)) { std::fprintf(stderr, "Payload invalido: %s\n", v.c_str()); return 1; }
        } else if (arg == "-h" || arg == "--help") {
            std::printf("Uso: %s [--mode=RX|TX|ALL] [--payload=0xFD020707]\n", argv[0]);
            return 0;
        }        else if (arg == "--leader") {
            if (i+1 >= argc) { std::fprintf(stderr, "Faltou valor para %s\n", arg.c_str()); return 1; }
            std::string v = argv[++i]; for (auto& c: v) c = std::tolower((unsigned char)c);
            if      (v == "full") g_leader_mode = LEADER_FULL;
            else if (v == "half") g_leader_mode = LEADER_HALF;
            else { std::fprintf(stderr, "Leader invalido: %s (use full|half)\n", v.c_str()); return 1; }
        } else if (arg.rfind("--leader=",0) == 0) {
            std::string v = eat("--leader="); for (auto& c: v) c = std::tolower((unsigned char)c);
            if      (v == "full") g_leader_mode = LEADER_FULL;
            else if (v == "half") g_leader_mode = LEADER_HALF;
            else { std::fprintf(stderr, "Leader invalido: %s (use full|half)\n", v.c_str()); return 1; }
        } else if (arg == "-c" || arg == "--command") {
            if (i+1 >= argc) { std::fprintf(stderr, "Faltou valor para %s\n", arg.c_str()); return 1; }
            g_cmd_name = argv[++i];
            g_mode = MODE_TX_ONESHOT;
        } else if (arg.rfind("--command=",0) == 0) {
            g_cmd_name = eat("--command=");
            g_mode = MODE_TX_ONESHOT;
        } else if (arg == "--map") {
            if (i+1 >= argc) { std::fprintf(stderr, "Faltou valor para %s\n", arg.c_str()); return 1; }
            g_map_path = argv[++i];
        } else if (arg.rfind("--map=",0) == 0) {
            g_map_path = eat("--map=");
        } else if (arg == "-h" || arg == "--help") {
            std::printf(
              "Uso: %s [--mode=RX|TX|ALL] [--payload=0xFD020707] [--leader=full|half]\n"
              "       %s -c/--command NOME [--map=ARQUIVO]\n"
              "Ex.:  %s -c TOGGLE --map=/etc/ir-controller/button-map\n",
              argv[0], argv[0], argv[0]);
            return 0;
        } 
    }
    if (!g_cmd_name.empty()) {
        // tentar --payload (se veio) primeiro; se nao veio, procurar no map
        if (payload_le == 0) {
            if (!load_map_payload(g_map_path, g_cmd_name, payload_le)) {
                // fallback: tentar ./button-map
                if (!load_map_payload("./button-map", g_cmd_name, payload_le)) {
                    std::fprintf(stderr, "Nao achei comando '%s' em '%s' nem em ./button-map\n",
                                 g_cmd_name.c_str(), g_map_path.c_str());
                    return 1;
                }
            }
        }
    }
    
    if (payload_le != 0) apply_payload_le(payload_le);

    if (g_leader_mode == LEADER_FULL) {
        NEC_LEADER_MARK_US  = 9000;
        NEC_LEADER_SPACE_US = 4500;
    } else { // LEADER_HALF
        NEC_LEADER_MARK_US  = 4500;
        NEC_LEADER_SPACE_US = 4500;
    }
    
    if (!ensure_backend_loaded()) {
        std::fprintf(stderr, "%s\n", g_backend_error.c_str());
        return 1;
    }

    if (!g_backend_error.empty()) {
        std::fprintf(stderr, "%s\n", g_backend_error.c_str());
        g_backend_error.clear();
    }

    if (gpioInitialise() < 0) {
        std::fprintf(stderr, "Falha ao inicializar biblioteca GPIO (precisa sudo)\n");
        return 1;
    }
        // caminho one-shot: manda e sai
    if (g_mode == MODE_TX_ONESHOT) {
        int rc = tx_once_and_exit();
        gpioTerminate();
        return rc;
    }

    std::signal(SIGINT,  on_sigint);
    std::signal(SIGTERM, on_sigint);

    // RX (somente quando escolhido)
    if (g_mode == MODE_ALL || g_mode == MODE_RX) {
        // zera estado RX
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            g_cur_pulses.clear();
        }
        g_last_tick       = 0;
        g_last_level      = 1;
        g_last_tick_atomic.store(0, std::memory_order_relaxed);

        // inicia RX
        GPIO_RX = GPIO_RX; // (no-op, apenas reforço)
        rx_start();
    }

    // TX (somente quando escolhido)
    std::thread th_tx;
    if (g_mode == MODE_ALL || g_mode == MODE_TX) {
       button_start(); 
       th_tx = std::thread(tx_thread);
    }

    // Loop principal ocioso
    while (g_running.load()) std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (th_tx.joinable()) th_tx.join();
    // desarma timer, callback e finaliza pigpio
    gpioSetTimerFuncEx(0, 0, nullptr, nullptr);
    gpioSetAlertFunc(GPIO_RX, nullptr);
    gpioTerminate();
    return 0;
}









