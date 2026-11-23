#include "nav/Gnss/NmeaRmc.h"

#include <array>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <type_traits>

namespace nav::gnss
{
	namespace
	{
		constexpr int DecimalBase = 10;
		constexpr int HexBase = 16;
		constexpr int ChecksumDigits = 2;
		constexpr int LatitudeDegreeDigits = 2;
		constexpr int LongitudeDegreeDigits = 3;
		constexpr int MinuteDigits = 2;
		constexpr int FractionalMinuteDigits = 6;
		constexpr int FractionalMinuteScale = 1'000'000;
		constexpr int ArcMinutesPerDegree = 60;
		constexpr int MaxLatitudeDegrees = 90;
		constexpr int MaxLongitudeDegrees = 180;
		constexpr int DateTimeFieldDigits = 2;
		constexpr int DateTimeDigits = 3 * DateTimeFieldDigits;
		constexpr int MillisecondDigits = 3;
		constexpr int MillisecondsPerSecond = 1000;
		constexpr int HoursPerDay = 24;
		constexpr int MinutesPerHour = 60;
		constexpr int SecondsPerMinute = 60;
		constexpr int YearsPerCentury = 100;
		constexpr int CenturyPivotYear = 80;
		constexpr int PreviousCentury = 1900;
		constexpr int CurrentCentury = 2000;
		constexpr int SentenceIdLength = 5;
		constexpr std::size_t SentenceIdField = 0;
		constexpr std::size_t TimeField = 1;
		constexpr std::size_t StatusField = 2;
		constexpr std::size_t LatitudeField = 3;
		constexpr std::size_t LatitudeHemisphereField = 4;
		constexpr std::size_t LongitudeField = 5;
		constexpr std::size_t LongitudeHemisphereField = 6;
		constexpr std::size_t DateField = 9;
		constexpr std::size_t RequiredFieldCount = DateField + 1;

		// Parses a complete numeric field from an RMC sentence.
		template <typename Number> bool parseNumber(std::string_view value, Number& number, int base = DecimalBase)
		{
			const auto* end = value.data() + value.size();
			if constexpr (std::is_integral_v<Number>)
			{
				const auto result = std::from_chars(value.data(), end, number, base);
				return result.ec == std::errc{} && result.ptr == end;
			}
			else
			{
				const auto result = std::from_chars(value.data(), end, number);
				return result.ec == std::errc{} && result.ptr == end && std::isfinite(number);
			}
		}

		// Computes the XOR checksum used to write or validate an RMC sentence.
		unsigned checksum(std::string_view payload)
		{
			unsigned value = 0;
			for (auto character : payload)
				value ^= static_cast<unsigned char>(character);
			return value;
		}

		// Writes one geographic coordinate in NMEA degrees-and-minutes format.
		void writeCoordinate(std::ostream& stream, double coordinate, int width, char positive, char negative)
		{
			// Round at the NMEA precision before splitting degrees and minutes so 59.999999 rounds correctly.
			const auto minuteUnits = std::llround(std::abs(coordinate) * ArcMinutesPerDegree * FractionalMinuteScale);
			stream << std::setw(width) << minuteUnits / (ArcMinutesPerDegree * FractionalMinuteScale) << std::setw(MinuteDigits)
				   << (minuteUnits / FractionalMinuteScale) % ArcMinutesPerDegree << '.' << std::setw(FractionalMinuteDigits)
				   << minuteUnits % FractionalMinuteScale << ',' << (coordinate < 0 ? negative : positive);
		}

		// Validates a coordinate field and converts it to signed decimal degrees.
		std::optional<double> readCoordinate(std::string_view value, std::string_view hemisphere, bool latitude)
		{
			const std::size_t digits = latitude ? LatitudeDegreeDigits : LongitudeDegreeDigits;
			const auto positive = latitude ? "N" : "E";
			const auto negative = latitude ? "S" : "W";
			int degrees = 0;
			double minutes = 0;
			if (value.size() < digits + MinuteDigits || !parseNumber(value.substr(0, digits), degrees) ||
				!parseNumber(value.substr(digits), minutes) || degrees < 0 || minutes < 0 || minutes >= ArcMinutesPerDegree ||
				(hemisphere != positive && hemisphere != negative))
				return std::nullopt;
			const auto coordinate = degrees + minutes / ArcMinutesPerDegree;
			if (coordinate > (latitude ? MaxLatitudeDegrees : MaxLongitudeDegrees))
				return std::nullopt;
			return hemisphere == negative ? -coordinate : coordinate;
		}
	}

	// Formats a simulated fix as an NMEA RMC sentence with a checksum.
	std::string encodeRmc(const RmcFix& fix)
	{
		const auto day = std::chrono::floor<std::chrono::days>(fix.measuredAt);
		const std::chrono::year_month_day date{day};
		const std::chrono::hh_mm_ss time(fix.measuredAt - day);
		std::ostringstream stream;
		stream.imbue(std::locale::classic());
		stream << "GNRMC," << std::setfill('0') << std::setw(DateTimeFieldDigits) << time.hours().count()
			   << std::setw(DateTimeFieldDigits) << time.minutes().count() << std::setw(DateTimeFieldDigits)
			   << time.seconds().count() << '.' << std::setw(MillisecondDigits) << time.subseconds().count() << ",A,";
		writeCoordinate(stream, fix.degrees.y, LatitudeDegreeDigits, 'N', 'S');
		stream << ',';
		writeCoordinate(stream, fix.degrees.x, LongitudeDegreeDigits, 'E', 'W');
		stream << ",,," << std::setw(DateTimeFieldDigits) << unsigned(date.day()) << std::setw(DateTimeFieldDigits)
			   << unsigned(date.month()) << std::setw(DateTimeFieldDigits) << int(date.year()) % YearsPerCentury << ",,,A";
		const auto payload = stream.str();
		stream.str({});
		stream << '$' << payload << '*' << std::uppercase << std::hex << std::setw(ChecksumDigits) << checksum(payload);
		return stream.str();
	}

	// Validates an RMC sentence and extracts its coordinates and UTC measurement time.
	std::optional<RmcFix> parseRmc(std::string_view sentence)
	{
		while (!sentence.empty() && (sentence.back() == '\r' || sentence.back() == '\n'))
			sentence.remove_suffix(1);
		const auto star = sentence.find('*');
		unsigned expected = 0;
		if (sentence.empty() || sentence.front() != '$' || star == std::string_view::npos ||
			sentence.size() - star != ChecksumDigits + 1 || !parseNumber(sentence.substr(star + 1), expected, HexBase))
			return std::nullopt;
		auto payload = sentence.substr(1, star - 1);
		if (checksum(payload) != expected)
			return std::nullopt;
		std::array<std::string_view, RequiredFieldCount> fields;
		for (auto& field : fields)
		{
			const auto comma = payload.find(',');
			field = payload.substr(0, comma);
			payload.remove_prefix(comma == std::string_view::npos ? payload.size() : comma + 1);
		}
		if (fields[SentenceIdField].size() != SentenceIdLength || !fields[SentenceIdField].ends_with("RMC") ||
			fields[StatusField] != "A" || fields[TimeField].size() < DateTimeDigits ||
			fields[DateField].size() != DateTimeDigits)
			return std::nullopt;
		const auto latitude = readCoordinate(fields[LatitudeField], fields[LatitudeHemisphereField], true);
		const auto longitude = readCoordinate(fields[LongitudeField], fields[LongitudeHemisphereField], false);
		int hour = 0, minute = 0, day = 0, month = 0, yearNumber = 0;
		double second = 0;
		if (!latitude || !longitude || !parseNumber(fields[TimeField].substr(0, DateTimeFieldDigits), hour) ||
			!parseNumber(fields[TimeField].substr(DateTimeFieldDigits, DateTimeFieldDigits), minute) ||
			!parseNumber(fields[TimeField].substr(2 * DateTimeFieldDigits), second) ||
			!parseNumber(fields[DateField].substr(0, DateTimeFieldDigits), day) ||
			!parseNumber(fields[DateField].substr(DateTimeFieldDigits, DateTimeFieldDigits), month) ||
			!parseNumber(fields[DateField].substr(2 * DateTimeFieldDigits, DateTimeFieldDigits), yearNumber) || hour < 0 ||
			hour >= HoursPerDay || minute < 0 || minute >= MinutesPerHour || second < 0 || second >= SecondsPerMinute ||
			day < 1 || month < 1 || yearNumber < 0)
			return std::nullopt;
		const std::chrono::year_month_day date{
			std::chrono::year{(yearNumber >= CenturyPivotYear ? PreviousCentury : CurrentCentury) + yearNumber},
			std::chrono::month(unsigned(month)),
			std::chrono::day(unsigned(day))};
		if (!date.ok())
			return std::nullopt;
		return RmcFix{{*longitude, *latitude},
					  std::chrono::sys_days(date) + std::chrono::hours(hour) + std::chrono::minutes(minute) +
						  std::chrono::milliseconds(std::llround(second * MillisecondsPerSecond))};
	}
}    // namespace nav::gnss
