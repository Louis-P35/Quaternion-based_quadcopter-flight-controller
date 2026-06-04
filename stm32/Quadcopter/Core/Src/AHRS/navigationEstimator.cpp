/*
 * navigationEstimator.cpp
 *
 *  Created on: Jun 1, 2026
 *      Author: louis
 */

#include "AHRS/navigationEstimator.hpp"
#include <cmath>

static constexpr float G       = 9.81f;   // m/s² — gravité standard
static constexpr float MIN_DT  = 0.0001f; // 100 µs — plancher pour éviter div/0
static constexpr float MAX_DT  = 0.05f;   // 50 ms  — au-delà, pas trop grand → clamper

// ============================================================================
// init
// ============================================================================

void NavigationEstimator::init(const NavParams& params)
{
    m_params     = params;
    m_N = m_E = m_D = Axis1D{};
    m_groundDist = 0.f;
    m_originSet  = false;
    m_lat0 = m_lon0 = 0.0;
    m_gpsAlt0    = 0.f;
    m_baroRefSet = false;
    m_baroRef    = 0.f;
    m_lidarPrev       = -1.f;
    m_lidarDtAcc      = 0.f;
    m_timeSinceGpsS   = 0.f;
}

// ============================================================================
// update — appelé à la fréquence IMU
// ============================================================================

void NavigationEstimator::update(const NavMeasurement& meas)
{
    // Clamp dt pour éviter explosion de la covariance sur un pas trop grand
    const float dt = (meas.dt < MIN_DT) ? MIN_DT
                   : (meas.dt > MAX_DT) ? MAX_DT
                   : meas.dt;

    // ── 1. PRÉDICTION : accéléromètre body → NED, soustraction gravité ──────
    //
    // L'accéléromètre est fourni en g → conversion en m/s² (SI) avant tout traitement.
    // Vérification hovering (quaternion identité) :
    //   az_b ≈ −1.0 g × 9.81 = −9.81 m/s² → bodyToNed → −9.81 sur Down → +G → 0 ✓
    static constexpr float G_TO_MS2 = 9.81f;
    const float ax_ms2 = meas.ax_b * G_TO_MS2;
    const float ay_ms2 = meas.ay_b * G_TO_MS2;
    const float az_ms2 = meas.az_b * G_TO_MS2;

    const Vector3<float> acc_ned = bodyToNed(meas.qw, meas.qx, meas.qy, meas.qz,
                                              ax_ms2, ay_ms2, az_ms2);
    const float a_n = acc_ned.m_x;
    const float a_e = acc_ned.m_y;
    const float a_d = acc_ned.m_z + G;  // G=9.81 m/s², cohérent : tout est en SI

    const float qh_sq = m_params.q_acc_h * m_params.q_acc_h;
    const float qv_sq = m_params.q_acc_v * m_params.q_acc_v;
    predictAxis(m_N, a_n, dt, qh_sq, m_params.q_pos_drift_h);
    predictAxis(m_E, a_e, dt, qh_sq, m_params.q_pos_drift_h);
    predictAxis(m_D, a_d, dt, qv_sq, m_params.q_pos_drift_v);

    // Accumulation des compteurs temporels inter-capteurs
    m_lidarDtAcc    += dt;
    m_timeSinceGpsS += dt;
    // [Point A] Si aucune lecture lidar depuis trop longtemps, invalider le gating :
    // on ne sait pas ce qui s'est passé pendant la fenêtre → prochaine lecture non gatée.
    static constexpr float LIDAR_GAP_MAX = 0.5f;  // 500 ms sans lidar = reset
    if (m_lidarDtAcc > LIDAR_GAP_MAX)
    {
        m_lidarPrev  = -1.f;
        m_lidarDtAcc = LIDAR_GAP_MAX;  // plafonné pour éviter la croissance infinie
    }

    // ── 2. CORRECTION GPS ─────────────────────────────────────────────────────
    //
    // Limitation connue : le fix GPS arrive via SPI depuis l'ESP32 avec un retard
    // non horodaté (latence SPI + traitement ~5–15 ms). L'erreur de position
    // induite (< 0.15 m à 10 m/s) est négligeable devant R_gps, mais ce système
    // n'est pas temps-réel strict sur la donnée GPS.
    //
    // R modulé par HDOP² : σ_gps ∝ HDOP → R = r_base × HDOP².
    // À HDOP=1 : R = r_gps_pos ; à HDOP=3 : R = 9 × r_gps_pos.
    if (meas.gps_new
        && meas.gps_valid
        && meas.gps_sats >= m_params.gps_min_sats
        && meas.gps_hdop <= m_params.gps_max_hdop)
    {
        if (!m_originSet)
        {
            // Premier fix valide : capture l'origine de la tangent plane
            m_lat0      = meas.gps_lat;
            m_lon0      = meas.gps_lon;
            m_gpsAlt0   = meas.gps_alt_m;
            m_originSet = true;

            // On est à l'origine NED (0, 0). Correction avec z=0 ancre la position.
            correctPos(m_N, 0.f, m_params.r_gps_pos);
            correctPos(m_E, 0.f, m_params.r_gps_pos);
        }
        else
        {
            float north_m, east_m;
            latLonToLocal(meas.gps_lat, meas.gps_lon, north_m, east_m);

            const float r_gps = m_params.r_gps_pos
                                * meas.gps_hdop * meas.gps_hdop;
            correctPos(m_N, north_m, r_gps);
            correctPos(m_E, east_m,  r_gps);
        }

        // GPS altitude : très bruité, R élevé → contribution faible.
        const float gps_down = -(meas.gps_alt_m - m_gpsAlt0);
        correctPos(m_D, gps_down, m_params.r_gps_alt);

        // Correction GPS appliquée → ancrage récent, timer reset
        m_timeSinceGpsS = 0.f;
    }

    // ── 3. CORRECTION BAROMÈTRE (source principale axe vertical) ─────────────
    //
    // Stratégie : le baro corrige la POSITION Down exclusivement.
    // On ne dérive pas le baro pour obtenir vz — son bruit haute fréquence
    // contaminerait la vitesse. La vitesse verticale est estimée par intégration
    // de l'accéléromètre + corrections lidar/GPS.
    //
    // [Correction C] Le baro corrige posDown, il ne touche pas groundDist.
    if (meas.baro_new)
    {
        if (!m_baroRefSet)
        {
            // Capture le zéro baro (altitude au sol au moment de l'init)
            m_baroRef    = meas.baro_alt_m;
            m_baroRefSet = true;
            // Pas de correction au premier échantillon : on fixe juste la référence
        }
        else
        {
            // baro_alt positif = en hauteur → Down NED = négatif
            const float baro_down = -(meas.baro_alt_m - m_baroRef);
            correctPos(m_D, baro_down, m_params.r_baro);
        }
    }

    // ── 4. LIDAR MTF-01 ───────────────────────────────────────────────────────
    //
    // Le lidar mesure la distance physique au sol → groundDist (état dédié).
    // groundDist et posDown sont deux états DISTINCTS et indépendants :
    //   - groundDist = hauteur au-dessus du sol physique sous le drone.
    //   - posDown    = altitude absolue dans le repère NED (ancré par baro/GPS).
    // Si le terrain n'est pas à l'altitude de l'origine, lidar ≠ −posDown.
    //
    // Correction posDown via lidar : désactivée par défaut (lidar_aids_altitude=false).
    // Raison : un offset constant de terrain (sol surélevé/abaissé par rapport à
    // l'origine) tirerait posDown vers une valeur fausse avec r_lidar très petit
    // (K ≈ 1), sans que le gating terrain puisse le détecter (pas de saut → gate ok).
    // Le gating ne protège que contre les discontinuités, pas les offsets permanents.
    //
    // Si activée (lidar_aids_altitude=true) : correction uniquement si terrain_ok
    // ET à basse altitude (lidar_altitude_max_m), avec R nettement plus élevé
    // que r_lidar (r_lidar_altitude). Réserver au cas where sol == origine garanti.
    const bool lidar_in_range = meas.lidar_new
                                && meas.lidar_dist_m > 0.05f
                                && meas.lidar_dist_m < m_params.lidar_max_dist_m;

    if (lidar_in_range)
    {
        // groundDist mis à jour systématiquement — indépendant de posDown
        m_groundDist = meas.lidar_dist_m;

        // Gating terrain : cohérence Δlidar avec velDown sur la fenêtre inter-lidar.
        // Sur terrain plat : Δlidar ≈ −velDown × Δt (descendre = Down+ = lidar ↓).
        bool terrain_ok = true;
        if (m_lidarPrev > 0.f && m_lidarDtAcc > 0.f)
        {
            const float expected_delta = -m_D.vel * m_lidarDtAcc;
            const float actual_delta   = meas.lidar_dist_m - m_lidarPrev;
            if (fabsf(actual_delta - expected_delta)
                    > m_params.lidar_terrain_gate * m_lidarDtAcc)
            {
                terrain_ok = false;
            }
        }

        m_lidarPrev  = meas.lidar_dist_m;
        m_lidarDtAcc = 0.f;

        // Correction posDown optionnelle — désactivée par défaut
        if (m_params.lidar_aids_altitude
            && terrain_ok
            && meas.lidar_dist_m < m_params.lidar_altitude_max_m)
        {
            const float lidar_down = -meas.lidar_dist_m;
            correctPos(m_D, lidar_down, m_params.r_lidar_altitude);
        }
    }
    else if (meas.lidar_new)
    {
        // Hors portée ou invalide : groundDist estimée depuis posDown
        // (suppose terrain plat au niveau de l'origine — approximation takeoff)
        m_groundDist = -m_D.pos;
        m_lidarPrev  = -1.f;
        m_lidarDtAcc = 0.f;
    }

    // ── 5. FLUX OPTIQUE MTF-01 ────────────────────────────────────────────────
    //
    // Unité : flow_vel = cm/s@1m (doc MicoAir officielle).
    //   speed(m/s) = flow_vel × height(m) / 100
    //
    // Dérotation gyro : IMPOSSIBLE directement — flow_vel n'est pas un angle
    // (rad) ni un taux angulaire (rad/s). MicoAir ne fournit aucune correction
    // angulaire dans le protocole. La rotation est gérée par deux mécanismes :
    //   1. Gating dur  : skip si |gyro_xy| > flow_max_gyro_rps.
    //   2. Pondération : R_flow ×= (1 + k × |gyro_xy|).
    //
    // Mapping axes (convention à valider au banc) :
    //   mouvement AVANT (v_body_x) ↔ +flow_vel_y × h / 100
    //   mouvement DROITE (v_body_y) ↔ −flow_vel_x × h / 100
    // TODO : valider — glisser vers l'avant à ~1 m/s sur surface texturée,
    //   vérifier que velNorth augmente (drone face au Nord). Idem droite → velEast.
    //
    // Hauteur : UNIQUEMENT groundDist (lidar valide). Pas de fallback sur −posDown
    // (un posDown dérivé polluerait le scaling et la correction de vitesse).
    {
        const float gyro_mag_xy = sqrtf(meas.gx_b * meas.gx_b
                                        + meas.gy_b * meas.gy_b);

        if (meas.flow_new
            && meas.flow_quality >= m_params.flow_min_quality
            && m_groundDist > 0.05f
            && gyro_mag_xy <= m_params.flow_max_gyro_rps)
        {
            const float height = m_groundDist;

            // Conversion cm/s@1m → m/s avec mapping croisé + signe
            const float v_body_x = ( meas.flow_vel_y) * height / 100.f;
            const float v_body_y = (-meas.flow_vel_x) * height / 100.f;

            // Rotation body → NED (couplage attitude obligatoire)
            const Vector3<float> v_ned = bodyToNed(meas.qw, meas.qx, meas.qy, meas.qz,
                                                    v_body_x, v_body_y, 0.f);

            // R de base dépendant de la hauteur, puis dégradé par la rotation
            const float sigma_v  = height * m_params.sigma_flow_rad;
            const float r_base   = sigma_v * sigma_v;
            const float r_flow   = r_base * (1.f + m_params.flow_gyro_r_scale * gyro_mag_xy);

            correctVel(m_N, v_ned.m_x, r_flow);
            correctVel(m_E, v_ned.m_y, r_flow);
        }
    }
}

// ============================================================================
// predictAxis — prédiction Kalman 1D (2 états : position, vitesse)
// ============================================================================

void NavigationEstimator::predictAxis(Axis1D& ax, float accel, float dt,
                                       float q_acc_sq, float q_pos_drift) const
{
    // Propagation d'état : intégration Euler d'ordre 2
    // [pos]   [1  dt] [pos]   [dt²/2]
    // [vel] = [0   1] [vel] + [dt   ] × accel
    ax.pos += ax.vel * dt + 0.5f * accel * dt * dt;
    ax.vel += accel * dt;

    // Propagation de la covariance : P = F·P·Fᵀ + Q
    // F = [[1, dt], [0, 1]]
    // F·P·Fᵀ :
    //   p00' = p00 + 2·p01·dt + p11·dt²
    //   p01' = p01 + p11·dt
    //   p11' = p11  (inchangé par F seul)
    // Q discret (termes dominants à dt = 1–2 ms) :
    //   Q_pos = q_pos_drift·dt   (drift de biais : dominant sur la position)
    //   Q_vel = q_acc_sq·dt      (bruit accéléromètre intégré)
    //   Q_cross ≈ q_acc_sq·dt²/2 ≈ 0 à dt=2 ms → ignoré
    const float dt2 = dt * dt;
    ax.p00 += 2.f * ax.p01 * dt + ax.p11 * dt2 + q_pos_drift * dt;
    ax.p01 += ax.p11 * dt;
    ax.p11 += q_acc_sq * dt;

    // Clamp pour stabilité numérique (P doit rester positive-définie)
    if (ax.p00 < 1e-6f) ax.p00 = 1e-6f;
    if (ax.p11 < 1e-6f) ax.p11 = 1e-6f;
}

// ============================================================================
// correctPos — correction Kalman sur mesure de POSITION  H = [1, 0]
// ============================================================================

void NavigationEstimator::correctPos(Axis1D& ax, float z_pos, float R) const
{
    const float innov = z_pos - ax.pos;
    const float S     = ax.p00 + R;  // variance de l'innovation

    if (!gated(innov, S)) return;

    const float K0 = ax.p00 / S;  // gain Kalman position
    const float K1 = ax.p01 / S;  // gain Kalman vitesse (via covariance croisée)

    ax.pos += K0 * innov;
    ax.vel += K1 * innov;

    // Mise à jour covariance P = (I − K·H)·P  pour H=[1,0] :
    //   p00_new = (1−K0)·p00 = R/S · p00
    //   p01_new = (1−K0)·p01 = R/S · p01
    //   p11_new = p11 − K1·p01 = p11 − p01²/S
    // Attention : calculer p11 AVANT de modifier p01 (utilise l'ancienne valeur)
    ax.p11 -= K1 * ax.p01;
    const float f = 1.f - K0;  // = R/S
    ax.p00 *= f;
    ax.p01 *= f;

    if (ax.p00 < 1e-6f) ax.p00 = 1e-6f;
    if (ax.p11 < 1e-6f) ax.p11 = 1e-6f;
}

// ============================================================================
// correctVel — correction Kalman sur mesure de VITESSE  H = [0, 1]
// ============================================================================

void NavigationEstimator::correctVel(Axis1D& ax, float z_vel, float R) const
{
    const float innov = z_vel - ax.vel;
    const float S     = ax.p11 + R;

    if (!gated(innov, S)) return;

    const float K0 = ax.p01 / S;  // gain pour position (via covariance croisée p01=p10)
    const float K1 = ax.p11 / S;  // gain pour vitesse

    ax.pos += K0 * innov;
    ax.vel += K1 * innov;

    // Pour H=[0,1] :
    //   p00_new = p00 − p01²/S
    //   p01_new = R/S · p01
    //   p11_new = R/S · p11
    // Calculer p00 AVANT de modifier p01 (utilise l'ancienne valeur)
    ax.p00 -= K0 * ax.p01;
    const float f = 1.f - K1;  // = R/S
    ax.p11 *= f;
    ax.p01 *= f;

    if (ax.p00 < 1e-6f) ax.p00 = 1e-6f;
    if (ax.p11 < 1e-6f) ax.p11 = 1e-6f;
}

// ============================================================================
// gated — test chi² à 1 degré de liberté
// ============================================================================

bool NavigationEstimator::gated(float innov, float S) const
{
    // Accepter si innovation² ≤ gate × S.
    // innov / sqrt(S) suit N(0,1) pour une mesure valide.
    // innov_gate = 9 → seuil 3σ → probabilité de faux-rejet = 0.27 %.
    return (innov * innov) <= (m_params.innov_gate * S);
}

// ============================================================================
// bodyToNed — rotation via produit en sandwich quaternion
// ============================================================================

Vector3<float> NavigationEstimator::bodyToNed(
    float qw, float qx, float qy, float qz,
    float vx, float vy, float vz)
{
    // v_NED = q ⊗ [0, v_body] ⊗ q*
    // Convention : q est le quaternion body → NED (sortie Madgwick).
    // La classe Quaternion<float> (Utils/quaternion.hpp) fournit operator*
    // et conjugate() — on s'appuie dessus directement.
    const Quaternion<float> q(qw, qx, qy, qz);
    const Quaternion<float> p(0.f, vx, vy, vz);
    const Quaternion<float> r = q * p * q.conjugate();
    return Vector3<float>(r.m_x, r.m_y, r.m_z);
}

// ============================================================================
// latLonToLocal — conversion flat-earth (lat, lon) → (north_m, east_m)
// ============================================================================

void NavigationEstimator::latLonToLocal(double lat, double lon,
                                         float& north_m, float& east_m) const
{
    // Approximation plan tangent (flat-earth) en (m_lat0, m_lon0).
    // Hypothèse : distances < ~20 km depuis l'origine → erreur < 0.18 %.
    // Pour un quadricopter opérant sur quelques centaines de mètres, l'erreur
    // est de l'ordre du millimètre — entièrement négligeable.
    //
    // North : arc de méridien → invariant avec la longitude.
    // East  : arc de parallèle → dépend de cos(lat0) (les méridiens convergent
    //         vers les pôles, 1° de longitude vaut moins en mètres à haute latitude).
    static constexpr double R_EARTH = 6371000.0;     // rayon moyen Terre (m)
    static constexpr double DEG2RAD = M_PI / 180.0;

    north_m = static_cast<float>((lat - m_lat0) * DEG2RAD * R_EARTH);
    east_m  = static_cast<float>((lon - m_lon0) * DEG2RAD * R_EARTH
                                 * std::cos(m_lat0 * DEG2RAD));
}

// ============================================================================
// selfTest — valide la convention quaternion body→NED au démarrage
// ============================================================================

bool NavigationEstimator::selfTest()
{
    // Test 1 : quaternion identité — le vecteur doit être inchangé.
    const auto v1 = bodyToNed(1.f, 0.f, 0.f, 0.f, 1.f, 2.f, 3.f);
    if (fabsf(v1.m_x - 1.f) > 1e-5f ||
        fabsf(v1.m_y - 2.f) > 1e-5f ||
        fabsf(v1.m_z - 3.f) > 1e-5f)
        return false;

    // Test 2 : rotation de +90° en lacet (yaw) autour de l'axe NED Z (Down).
    // q = [cos(45°), 0, 0, sin(45°)] ≈ [√2/2, 0, 0, √2/2].
    // La pointe du drone (body X = avant) pointe à l'Est après 90° de lacet.
    // Attendu : bodyToNed([1,0,0]) → NED [0,1,0]  (North≈0, East≈1, Down≈0).
    static constexpr float SQ2_2 = 0.7071068f;
    const auto v2 = bodyToNed(SQ2_2, 0.f, 0.f, SQ2_2, 1.f, 0.f, 0.f);
    if (fabsf(v2.m_x)       > 1e-5f ||   // North ≈ 0
        fabsf(v2.m_y - 1.f) > 1e-5f ||   // East  ≈ 1
        fabsf(v2.m_z)       > 1e-5f)      // Down  ≈ 0
        return false;

    return true;
}
