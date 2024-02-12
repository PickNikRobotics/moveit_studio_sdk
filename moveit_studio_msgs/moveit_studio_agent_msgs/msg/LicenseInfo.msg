# All possible license states mapped to LicenseStatus enum in license_utils.hpp.
int8 ACTIVE = 0
int8 INACTIVE = 1
int8 SUSPENDED = 2
int8 EXPIRED = 3
int8 REVOKED = 4
int8 INET = 5
int8 UNKNOWN = 6

# Current license status.
int8 status

# All possible license types mapped to LicenseType enum in license_utils.hpp.
int8 FULL = 0
int8 TRIAL = 1
int8 OEM = 2
int8 UNLICENSED = 3

# Current license type.
int8 type

# Expiration date as POSIX time. Number of seconds elapsed since 00:00:00,
# coordinated Universal Time (UTC), Thursday, 1 January 1970, not counting leap seconds.
uint32 expiration_date

# True when the license if a floating one i.e. multiple seats per license.
bool is_floating

# The email tied to the license key
string email
