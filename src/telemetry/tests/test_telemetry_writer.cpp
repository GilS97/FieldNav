#include <gtest/gtest.h>

#include <fieldnav/telemetry/telemetry_writer.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace ft = fieldnav::telemetry;

namespace {

std::string tempFile(const std::string& name) {
    return ::testing::TempDir() + name;
}

std::vector<std::string> readLines(const std::string& file) {
    std::ifstream in(file, std::ios::binary);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line)) {
        lines.push_back(line);
    }
    return lines;
}

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string f;
    while (std::getline(ss, f, ',')) {
        fields.push_back(f);
    }
    if (!line.empty() && line.back() == ',') {
        fields.emplace_back(); // dernier champ vide
    }
    return fields;
}

ft::TelemetrySample sample(int i) {
    ft::TelemetrySample s;
    s.t_s = 0.1 * (i + 1);
    s.x_true = 0.032 * i;
    s.y_true = 0.0;
    s.th_true = 0.0;
    s.x_est = 0.032 * i + 0.001;
    s.y_est = -0.0002 * i;
    s.th_est = 0.0001 * i;
    if (i % 2 == 1) { // une mesure GPS une ligne sur deux (5 Hz pour 10 Hz de log)
        s.x_gps = 0.032 * i + 0.5;
        s.y_gps = -0.3;
    }
    s.v_cmd = 0.3;
    s.w_cmd = -0.01;
    s.lat_err = 0.0;
    return s;
}

} // namespace

TEST(TelemetryWriter, CsvHeaderMatchesDocumentedColumns) {
    const auto file = tempFile("header.csv");
    {
        ft::TelemetryWriter w(file, ft::TelemetryFormat::Csv);
    }
    const auto lines = readLines(file);
    ASSERT_EQ(lines.size(), 1U);
    EXPECT_EQ(lines[0], "t_s,x_true,y_true,th_true,x_est,y_est,th_est,x_gps,y_gps,gps_valid,"
                        "v_cmd,w_cmd,lat_err");
}

TEST(TelemetryWriter, WritesAndRereadsHundredSamples) {
    // Test de la tâche : écrire 100 échantillons, relire, comparer.
    const auto file = tempFile("run.csv");
    {
        ft::TelemetryWriter w(file, ft::TelemetryFormat::Csv);
        for (int i = 0; i < 100; ++i) {
            w.write(sample(i));
        }
        EXPECT_EQ(w.samples(), 100U);
    }
    const auto lines = readLines(file);
    ASSERT_EQ(lines.size(), 101U);
    for (int i = 0; i < 100; ++i) {
        const auto f = splitCsv(lines[static_cast<std::size_t>(i) + 1]);
        ASSERT_EQ(f.size(), 13U) << lines[static_cast<std::size_t>(i) + 1];
        const auto s = sample(i);
        EXPECT_NEAR(std::stod(f[0]), s.t_s, 5e-4);
        EXPECT_NEAR(std::stod(f[4]), s.x_est, 5e-5);
        EXPECT_NEAR(std::stod(f[6]), s.th_est, 5e-6);
        if (s.x_gps) {
            EXPECT_EQ(f[9], "1");
            EXPECT_NEAR(std::stod(f[7]), *s.x_gps, 5e-5);
        } else {
            EXPECT_EQ(f[9], "0");
            EXPECT_EQ(f[7], "");
            EXPECT_EQ(f[8], "");
        }
    }
}

TEST(TelemetryWriter, NoNegativeZero) {
    const auto file = tempFile("zero.csv");
    {
        ft::TelemetryWriter w(file, ft::TelemetryFormat::Csv);
        ft::TelemetrySample s;
        s.y_est = -0.00001; // s'arrondit à 0,0000 : doit être écrit « 0.0000 », pas « -0.0000 »
        w.write(s);
    }
    EXPECT_EQ(readLines(file)[1].find("-0.0000"), std::string::npos);
}

TEST(TelemetryWriter, JsonLinesUseSameKeysAndNull) {
    const auto file = tempFile("run.jsonl");
    EXPECT_EQ(ft::TelemetryWriter::formatFromPath(file), ft::TelemetryFormat::JsonLines);
    {
        ft::TelemetryWriter w(file, ft::TelemetryWriter::formatFromPath(file));
        w.write(sample(0)); // sans GPS
        w.write(sample(1)); // avec GPS
    }
    const auto lines = readLines(file);
    ASSERT_EQ(lines.size(), 2U);
    EXPECT_EQ(lines[0].front(), '{');
    EXPECT_EQ(lines[0].back(), '}');
    EXPECT_NE(lines[0].find("\"x_gps\":null"), std::string::npos);
    EXPECT_NE(lines[0].find("\"gps_valid\":0"), std::string::npos);
    EXPECT_NE(lines[1].find("\"gps_valid\":1"), std::string::npos);
    EXPECT_NE(lines[1].find("\"lat_err\":"), std::string::npos);
}

TEST(TelemetryWriter, CsvByDefault) {
    EXPECT_EQ(ft::TelemetryWriter::formatFromPath("out/run.csv"), ft::TelemetryFormat::Csv);
    EXPECT_EQ(ft::TelemetryWriter::formatFromPath("run"), ft::TelemetryFormat::Csv);
}

TEST(TelemetryWriter, SameSamplesSameBytes) {
    const auto a = tempFile("a.csv"), b = tempFile("b.csv");
    for (const auto& f : {a, b}) {
        ft::TelemetryWriter w(f, ft::TelemetryFormat::Csv);
        for (int i = 0; i < 20; ++i) {
            w.write(sample(i));
        }
    }
    EXPECT_EQ(readLines(a), readLines(b));
}

TEST(TelemetryWriter, UnwritablePathThrows) {
    EXPECT_THROW(ft::TelemetryWriter("dossier_inexistant/x/run.csv", ft::TelemetryFormat::Csv),
                 std::runtime_error);
}
