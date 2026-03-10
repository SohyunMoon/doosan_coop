 #pragma once
 
 #include <array>
 #include <cmath>
 #include <string>
 
 namespace SKKU {
 
 inline bool isValidElbow(const std::array<double, 2>& elbow) noexcept {
   return elbow[1] == -1.0 || elbow[1] == 1.0;
 }
 
 inline bool isHomogeneousTransformation(const std::array<double, 16>& transform) noexcept {
   constexpr double kOrthonormalThreshold = 1e-5;
 
   if (transform[3] != 0.0 || transform[7] != 0.0 || transform[11] != 0.0 || transform[15] != 1.0) {
     return false;
   }
   for (size_t j = 0; j < 3; ++j) {  // i..column
     if (std::abs(std::sqrt(std::pow(transform[j * 4 + 0], 2) + std::pow(transform[j * 4 + 1], 2) +
                            std::pow(transform[j * 4 + 2], 2)) -
                  1.0) > kOrthonormalThreshold) {
       return false;
     }
   }
   for (size_t i = 0; i < 3; ++i) {  // j..row
     if (std::abs(std::sqrt(std::pow(transform[0 * 4 + i], 2) + std::pow(transform[1 * 4 + i], 2) +
                            std::pow(transform[2 * 4 + i], 2)) -
                  1.0) > kOrthonormalThreshold) {
       return false;
     }
   }
   return true;
 }
 
 bool hasRealtimeKernel();
 
 bool setCurrentThreadToHighestSchedulerPriority(std::string* error_message);
 
 }