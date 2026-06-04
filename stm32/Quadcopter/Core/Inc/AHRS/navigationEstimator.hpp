/*
 * navigationEstimator.hpp
 *
 *  Created on: Jun 1, 2026
 *      Author: louis
 */

#pragma once

#include "Utils/quaternion.hpp"
#include "Utils/vector.hpp"
#include <cstdint>

// ============================================================================
// NavParams — tous les paramètres de réglage regroupés ici.
//
// Q (bruit de processus) : représente l'incertitude du modèle IMU.
//   q_acc_*      : densité de bruit RMS de l'accéléromètre (m/s²). Pour un
//                  quadricopter, la valeur effective est dominée par les vibrations
//                  moteur, pas par le datasheet du capteur. Valeur conservatrice.
//   q_pos_drift  : variance de position par seconde due au drift du biais accéléro.
//                  Empêche le filtre de sur-faire confiance au dead-reckoning.
//   → Augmenter si le filtre est trop rigide (suit mal les corrections GPS/baro).
//   → Diminuer si l'estimée est trop nerveuse (saute sur chaque bruit GPS).
//
// R (bruit de mesure, variances) :
//   r_gps_pos   : à HDOP=1. Modulé dynamiquement par HDOP² dans update().
//   r_baro      : précis court-terme (~0.1–0.3 m), mais dérive long-terme.
//   sigma_flow_rad : bruit angulaire du capteur de flux (rad/s). [Correction A]
//                    R_flow est recalculé à chaque update : R = (h × σ)².
//   r_lidar     : TFmini/MTF-01 : ~3 cm à courte portée.
//
// Gating et seuils :
//   innov_gate   : rejet chi² si innovation² > gate × S (S = variance prédite).
//                  9.0 = 3σ², compromis standard robustesse / réactivité.
//   lidar_terrain_gate : seuil en m/s. Si |Δlidar/Δt + velDown| > seuil, la
//                        variation du lidar est interprétée comme un saut de
//                        terrain, et la correction de posDown est inhibée.
// ============================================================================
struct NavParams
{
    // ── Bruit de processus ──────────────────────────────────────────────────
    float q_acc_h        = 0.5f;   // m/s²  — bruit accéléromètre axes horiz.
    float q_acc_v        = 0.3f;   // m/s²  — bruit accéléromètre axe vertical
    float q_pos_drift_h  = 0.02f;  // m²/s  — drift de biais position, horiz.
    float q_pos_drift_v  = 0.01f;  // m²/s  — drift de biais position, vertical

    // ── Bruit de mesure (variances) ─────────────────────────────────────────
    float r_gps_pos      = 6.25f;  // m²       — GPS horiz. à HDOP=1 (σ=2.5 m)
    float r_gps_alt      = 25.0f;  // m²       — GPS altitude (σ=5 m, très bruité)
    float r_baro         = 0.04f;  // m²       — baromètre relatif (σ=0.2 m)
    // [Correction A] R_flow n'est pas constant : R = (h × sigma_flow_rad)²
    float sigma_flow_rad = 0.05f;  // rad/s — bruit angulaire du flux MTF-01
    float r_lidar        = 0.001f; // m²       — lidar hauteur-sol (σ=0.03 m)

    // ── Seuils de validité et gating ────────────────────────────────────────
    float   innov_gate         = 9.0f;  // chi² gate : rejeter si innov² > gate×S
    uint8_t gps_min_sats       = 6;     // satellites minimum pour utiliser GPS
    float   gps_max_hdop       = 2.5f;  // HDOP maximum acceptable
    uint8_t flow_min_quality   = 100;   // qualité flux MTF-01 minimum (0–255)
    // Gating rotation flux : flow_vel est en cm/s@1m, pas un angle → dérotation
    // gyro impossible en unités. La rotation est gérée par deux mécanismes :
    //   1. Gating dur  : si |gyro_xy| > flow_max_gyro_rps → skip correction flux.
    //   2. Pondération : R_flow ×= (1 + flow_gyro_r_scale × |gyro_xy|) → confiance
    //      réduite continûment quand le drone tourne.
    float   flow_max_gyro_rps  = 0.5f;  // rad/s — seuil gating dur (≈29°/s)
    float   flow_gyro_r_scale  = 5.0f;  // facteur pondération R (adimensionnel)
    float   lidar_max_dist_m   = 8.0f;  // au-delà : lidar hors portée → baro
    float   lidar_terrain_gate = 2.0f;  // m/s — seuil détection saut de terrain

    // ── Aide altitude par lidar (désactivée par défaut) ─────────────────────
    // Par défaut, le lidar ne corrige PAS posDown : groundDist (hauteur-sol) et
    // posDown (altitude absolue) sont deux états distincts. Un sol plat mais
    // surélevé par rapport à l'origine tirerait posDown vers une valeur fausse
    // avec r_lidar très petit (confiance énorme), sans que le gating terrain
    // puisse le détecter (offset constant ≠ saut).
    // Activer uniquement si le décollage se fait toujours depuis le sol de
    // référence ET à basse altitude (lidar_altitude_max_m).
    bool  lidar_aids_altitude  = false;  // activer manuellement si nécessaire
    float r_lidar_altitude     = 0.25f;  // m² — R pour cette correction (σ=0.5 m)
    float lidar_altitude_max_m = 3.0f;   // m  — correction inhibée au-dessus

    // ── Timeout de validité horizontale ─────────────────────────────────────
    // Après gps_horiz_timeout_s secondes sans correction GPS, horizontalValid()
    // repasse à false : la position dérive par dead-reckoning, le contrôleur
    // de position ne doit plus lui faire confiance.
    // 0 = pas de timeout (utile en simulation ou test sur banc).
    float gps_horiz_timeout_s = 30.f;   // secondes (0 = désactivé)
};

// ============================================================================
// NavMeasurement — données capteurs pour un appel update().
//
// Appeler update() à la fréquence IMU (typiquement 1–4 kHz). Pour les capteurs
// moins fréquents (GPS 1 Hz, baro 50 Hz, lidar/flux 100 Hz), positionner le
// flag _new = true uniquement lors du cycle où une nouvelle trame est disponible.
// Quand _new = false, la correction Kalman correspondante est skippée.
// ============================================================================
struct NavMeasurement
{
    float dt;  // secondes depuis le dernier appel (> 0, typiquement 0.25–2 ms)

    // ── Attitude + IMU (obligatoire à chaque appel) ──────────────────────────
    float qw, qx, qy, qz;          // quaternion unité : rotation body → NED
    float ax_b, ay_b, az_b;        // accéléromètre en g (repère body) — converti en m/s² dans update()
    float gx_b, gy_b, gz_b;        // gyroscope body frame, rad/s (dérotation flux)

    // ── GPS (gps_new = nouvelle trame reçue ce cycle) ────────────────────────
    // Note : les données GPS arrivent via SPI depuis l'ESP32 avec un retard
    // non horodaté (~quelques ms). L'erreur de position induite est négligeable
    // devant R_gps, mais ce système n'est pas temps-réel strict sur le GPS.
    bool    gps_new   = false;
    bool    gps_valid = false;
    double  gps_lat   = 0.0;       // degrés décimaux
    double  gps_lon   = 0.0;
    float   gps_alt_m = 0.f;       // altitude AMSL, mètres
    uint8_t gps_sats  = 0;
    float   gps_hdop  = 99.f;

    // ── MTF-01 flux optique ──────────────────────────────────────────────────
    // flow_vel_x/y : vitesse normalisée, unité cm/s@1m (doc MicoAir officielle).
    // Formule MicoAir : speed(cm/s) = flow_vel × height(m).
    // Ce n'est PAS un taux angulaire (rad/s) : dérotation gyro directe impossible
    // (unités incompatibles). La rotation est gérée par gating/pondération.
    bool    flow_new     = false;
    float   flow_vel_x   = 0.f;    // cm/s@1m, body frame (axe X capteur)
    float   flow_vel_y   = 0.f;    // cm/s@1m, body frame (axe Y capteur)
    uint8_t flow_quality = 0;      // 0–255

    // ── MTF-01 lidar ─────────────────────────────────────────────────────────
    bool  lidar_new    = false;
    float lidar_dist_m = 0.f;      // hauteur-sol mesurée, mètres (positif)

    // ── Baromètre ─────────────────────────────────────────────────────────────
    // Chemin entièrement implémenté, inactif tant que baro_new reste false.
    // Brancher ici quand le capteur sera disponible, sans réécriture.
    bool  baro_new    = false;
    float baro_alt_m  = 0.f;       // altitude relative, mètres (positif = en hauteur)
};

// ============================================================================
// NavigationEstimator — fusion Kalman 1D par axe (North, East, Down).
//
// Pourquoi Kalman et pas filtre complémentaire ?
//   Le complémentaire utilise des gains fixes (α, β) optimaux pour un ratio
//   signal/bruit constant et une paire de capteurs. Ici on fusionne 4 sources
//   à fréquences et fiabilités hétérogènes :
//     - GPS  : 1 Hz, 2–5 m, optionnel
//     - Baro : 50 Hz, 0.2 m, source principale verticale
//     - Lidar: 100 Hz, 3 cm, conditionnel (hors-portée > 8 m)
//     - Flux : 100 Hz, dépend de l'altitude
//   Le Kalman propage automatiquement l'incertitude (P grossit pendant les
//   absences, se réduit via K = P·Hᵀ/S sur chaque mesure). La pondération
//   est automatique : GPS HDOP=3 contribue 9× moins qu'HDOP=1 sans retoucher
//   les gains. Impossible à obtenir proprement avec un complémentaire.
//
// Architecture : 3 filtres 1D indépendants [pos, vel] sur North, East, Down.
//   Axes découplés : approximation valide tant que les accélérations couplées
//   entre axes sont petites (vrai pour un vol stabilisé). Réduit la complexité
//   de 6×6 (EKF full) à 3×2×2, sans dégradation notable en pratique quadricopter.
// ============================================================================
class NavigationEstimator
{
public:
    // Initialise les filtres. Peut être appelé pour réinitialiser en vol.
    void init(const NavParams& params = NavParams{});

    // Appeler à la fréquence IMU. Lance predict() à chaque appel, puis les
    // corrections Kalman uniquement pour les capteurs avec _new = true.
    void update(const NavMeasurement& meas);

    // ── Getters position NED (mètres depuis l'origine GPS) ──────────────────
    float posNorth()   const { return m_N.pos; }
    float posEast()    const { return m_E.pos; }
    float posDown()    const { return m_D.pos; }

    // ── Getters vitesse NED (m/s) ────────────────────────────────────────────
    float velNorth()   const { return m_N.vel; }
    float velEast()    const { return m_E.vel; }
    float velDown()    const { return m_D.vel; }

    // ── Hauteur-sol (m) ───────────────────────────────────────────────────────
    // [Correction C] État distinct de posDown : lidar donne la distance au sol
    // physique, qui peut différer de -posDown si le terrain n'est pas plat.
    // Fallback vers -posDown quand lidar est hors portée (terrain plat supposé).
    float groundDist() const { return m_groundDist; }

    // ── Statut ────────────────────────────────────────────────────────────────
    // true uniquement après premier fix GPS valide.
    bool isOriginSet() const { return m_originSet; }

    // true si la position horizontale a un ancrage absolu GPS ET que la dernière
    // correction GPS est récente (< gps_horiz_timeout_s).
    //
    // Sémantique pour le consommateur (contrôleur de position, hold mode) :
    //   false → posNorth()/posEast() dérivent : ne pas les utiliser comme
    //           référence absolue. En mode flux-seul indoor, seule la VITESSE
    //           est fiable ; la position dérive par intégration sans ancrage.
    //   true  → ancrage GPS actif et récent : position exploitable en boucle fermée.
    //
    // Le timeout (gps_horiz_timeout_s) protège contre le dead-reckoning prolongé
    // après une perte de signal GPS : même si m_originSet est true, la position
    // a pu dériver significativement depuis la dernière correction.
    bool horizontalValid() const
    {
        if (!m_originSet) return false;
        return m_params.gps_horiz_timeout_s <= 0.f
               || m_timeSinceGpsS <= m_params.gps_horiz_timeout_s;
    }

    // Vérifie la convention quaternion body→NED. À appeler au démarrage.
    // Retourne false si la rotation sandwich donne un résultat inattendu.
    static bool selfTest();

private:
    // ── État Kalman 1D ────────────────────────────────────────────────────────
    // P = [[p00, p01], [p01, p11]] symétrique → 3 scalaires suffisent.
    // Valeurs initiales : incertitude large pour converger rapidement dès les
    // premières mesures (p00 = 10 m², p11 = 1 (m/s)²).
    struct Axis1D {
        float pos = 0.f, vel = 0.f;
        float p00 = 10.f;  // variance position init (σ ≈ 3.2 m)
        float p11 =  1.f;  // variance vitesse init  (σ = 1 m/s)
        float p01 =  0.f;  // covariance croisée init (indépendants au départ)
    };

    Axis1D m_N, m_E, m_D;   // axes North, East, Down
    float  m_groundDist = 0.f;
    NavParams m_params;

    // ── Origine GPS (premier fix valide) ─────────────────────────────────────
    bool   m_originSet = false;
    double m_lat0      = 0.0;  // degrés décimaux
    double m_lon0      = 0.0;
    float  m_gpsAlt0   = 0.f;  // altitude AMSL à l'origine, mètres

    // ── Référence baromètre ───────────────────────────────────────────────────
    bool  m_baroRefSet = false;
    float m_baroRef    = 0.f;  // altitude baro au moment de l'init (zéro relatif)

    // ── Suivi lidar pour détection de saut de terrain ─────────────────────────
    float m_lidarPrev  = -1.f;  // < 0 = pas de lecture précédente (reset état)
    float m_lidarDtAcc = 0.f;   // dt cumulé depuis la dernière mesure lidar valide

    // ── Suivi de la fraîcheur de la correction GPS ────────────────────────────
    float m_timeSinceGpsS = 0.f;  // secondes depuis la dernière correction GPS valide

    // ── Méthodes internes ─────────────────────────────────────────────────────

    // Prédiction Kalman : propage [pos, vel] et la covariance P via l'accélération.
    // q_acc_sq   = q_acc² : densité de bruit accéléromètre au carré (m²/s⁴ · s = m²/s³)
    // q_pos_drift : variance de position par seconde (drift de biais, m²/s)
    void predictAxis(Axis1D& ax, float accel, float dt,
                     float q_acc_sq, float q_pos_drift) const;

    // Correction Kalman sur mesure de POSITION — H = [1, 0]
    void correctPos(Axis1D& ax, float z_pos, float R) const;

    // Correction Kalman sur mesure de VITESSE — H = [0, 1]
    void correctVel(Axis1D& ax, float z_vel, float R) const;

    // Gating chi² : true = mesure acceptée, false = outlier rejeté
    bool gated(float innov, float S) const;

    // Rotation body → NED par produit en sandwich : v_ned = q ⊗ [0,v] ⊗ q*
    // Utilise Quaternion<float> de Utils/quaternion.hpp (opérateur* disponible).
    static Vector3<float> bodyToNed(float qw, float qx, float qy, float qz,
                                    float vx, float vy, float vz);

    // Conversion flat-earth (lat, lon) → (north_m, east_m) depuis l'origine.
    // Hypothèse : distances < ~20 km (erreur < 0.18 % — suffisant quadricopter).
    void latLonToLocal(double lat, double lon,
                       float& north_m, float& east_m) const;
};
