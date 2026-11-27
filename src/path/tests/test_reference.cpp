#include <gtest/gtest.h>

#include <cmath>
#include <fieldnav/path/reference.hpp>
#include <fstream>
#include <stdexcept>
#include <string>

namespace fp = fieldnav::path;

namespace {

// Écrit un petit CSV temporaire et renvoie son chemin.
std::string writeTemp(const std::string& name, const std::string& content) {
    const std::string file = ::testing::TempDir() + name;
    std::ofstream(file) << content;
    return file;
}

const std::string kStraight = std::string(FIELDNAV_DATA_DIR) + "/reference_straight.csv";

} // namespace

// ─── Lecture (la boucle est à écrire par Gil) ───

TEST(ReferenceCsv, MissingFileThrows) {
    EXPECT_THROW((void)fp::loadReferenceCsv("n_existe_pas.csv"), std::runtime_error);
}

TEST(ReferenceCsv, ParsesSmallFile) {
    const auto pts = fp::loadReferenceCsv(writeTemp("small.csv", "s,x,y\n0,0,0\n0.1,0.1,0\n"));
    ASSERT_EQ(pts.size(), 2U);
    EXPECT_DOUBLE_EQ(pts[1].s, 0.1);
    EXPECT_DOUBLE_EQ(pts[1].x, 0.1);
    EXPECT_DOUBLE_EQ(pts[1].y, 0.0);
}

TEST(ReferenceCsv, IgnoresTrailingEmptyLine) {
    const auto pts = fp::loadReferenceCsv(writeTemp("trail.csv", "s,x,y\n0,0,0\n\n"));
    EXPECT_EQ(pts.size(), 1U);
}

TEST(ReferenceCsv, AcceptsWindowsLineEndings) {
    // Un CSV ouvert et réenregistré sous Windows a des fins de ligne « \r\n ».
    const auto pts = fp::loadReferenceCsv(writeTemp("crlf.csv", "s,x,y\r\n0,0,0\r\n0.1,0.1,0\r\n"));
    ASSERT_EQ(pts.size(), 2U);
    EXPECT_DOUBLE_EQ(pts[1].x, 0.1);
}

TEST(ReferenceCsv, MalformedLineThrowsWithLineNumber) {
    const auto file = writeTemp("bad.csv", "s,x,y\n0,0,0\n0.1,abc,0\n");
    try {
        (void)fp::loadReferenceCsv(file);
        FAIL() << "aucune exception";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("ligne 3"), std::string::npos) << e.what();
    }
}

TEST(ReferenceCsv, WrongColumnCountThrows) {
    EXPECT_THROW((void)fp::loadReferenceCsv(writeTemp("cols.csv", "s,x,y\n0,0\n")),
                 std::runtime_error);
    EXPECT_THROW((void)fp::loadReferenceCsv(writeTemp("cols4.csv", "s,x,y\n0,0,0,0\n")),
                 std::runtime_error);
}

// ─── Le fichier commité (généré par scripts/gen_row_scene.py) ───

TEST(ReferenceStraight, FiveHundredOnePointsEveryTenCentimetres) {
    const auto pts = fp::loadReferenceCsv(kStraight);
    ASSERT_EQ(pts.size(), 501U); // 50 m / 0,1 m = 500 intervalles
    EXPECT_DOUBLE_EQ(pts.front().s, 0.0);
    EXPECT_DOUBLE_EQ(pts.back().s, 50.0);
    for (std::size_t i = 1; i < pts.size(); ++i) {
        EXPECT_NEAR(pts[i].s - pts[i - 1].s, 0.1, 1e-9) << "à l'indice " << i;
    }
}

TEST(ReferenceStraight, IsTheXAxis) {
    for (const auto& p : fp::loadReferenceCsv(kStraight)) {
        EXPECT_NEAR(p.x, p.s, 1e-9);
        EXPECT_NEAR(p.y, 0.0, 1e-9);
    }
}

TEST(ReferenceStraight, ArcLengthMatchesDistanceBetweenPoints) {
    // s doit être la distance réellement parcourue : la somme des segments.
    const auto pts = fp::loadReferenceCsv(kStraight);
    double length = 0.0;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        length += std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
    }
    EXPECT_NEAR(length, pts.back().s, 1e-6);
}
