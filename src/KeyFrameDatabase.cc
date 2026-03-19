/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel
 * and Juan D. Tardós, University of Zaragoza. Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M.
 * Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU
 * General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "KeyFrameDatabase.h"
#include <DBoW2/BowVector.h>
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"

namespace ORB_SLAM3 {

KeyFrameDatabase::KeyFrameDatabase() = default;

KeyFrameDatabase::KeyFrameDatabase(const ORBVocabulary& voc) : mpVoc(&voc) {
  mvInvertedFile.resize(voc.size());
}

void KeyFrameDatabase::add(KeyFrame* pKF) {
  const std::unique_lock<std::mutex> lock(mMutex);

  for (const auto& [wordId, wordValue] : pKF->mBowVec) {
    mvInvertedFile[wordId].push_back(pKF);
  }
}

void KeyFrameDatabase::erase(KeyFrame* pKF) {
  const std::unique_lock<std::mutex> lock(mMutex);

  // Erase elements in the Inverse File for the entry
  for (const auto& [wordId, wordValue] : pKF->mBowVec) {
    // List of keyframes that share the word
    std::list<KeyFrame*>& lKFs = mvInvertedFile[wordId];

    for (auto lit = lKFs.begin(), lend = lKFs.end(); lit != lend; lit++) {
      if (pKF == *lit) {
        lKFs.erase(lit);
        break;
      }
    }
  }
}

void KeyFrameDatabase::clear() {
  mvInvertedFile.clear();
  mvInvertedFile.resize(mpVoc->size());
}

void KeyFrameDatabase::clearMap(Map* pMap) {
  const std::unique_lock<std::mutex> lock(mMutex);

  // Erase elements in the Inverse File for the entry
  for (auto& lKFs : mvInvertedFile) {
    for (auto lit = lKFs.begin(), lend = lKFs.end(); lit != lend;) {
      KeyFrame* pKFi = *lit;
      if (pMap == pKFi->GetMap()) {
        lit = lKFs.erase(lit);
        // Dont delete the KF because the class Map clean all the KF when it is destroyed
      } else {
        ++lit;
      }
    }
  }
}

vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(KeyFrame* pKF, float minScore) {
  const std::set<KeyFrame*> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
  std::list<KeyFrame*>      lKFsSharingWords;

  // Search all keyframes that share a word with current keyframes
  // Discard keyframes connected to the query keyframe
  {
    const std::unique_lock<std::mutex> lock(mMutex);

    for (const auto& [wordId, wordValue] : pKF->mBowVec) {
      std::list<KeyFrame*>& lKFs = mvInvertedFile[wordId];

      for (auto* pKFi : lKFs) {
        if (pKFi->GetMap() == pKF->GetMap()) { // For consider a loop candidate it a candidate it
                                               // must
                                               // be in the same map
          if (pKFi->mnLoopQuery != pKF->mnId) {
            pKFi->mnLoopWords = 0;
            if (!spConnectedKeyFrames.contains(pKFi)) {
              pKFi->mnLoopQuery = pKF->mnId;
              lKFsSharingWords.push_back(pKFi);
            }
          }
          pKFi->mnLoopWords++;
        }
      }
    }
  }

  if (lKFsSharingWords.empty()) {
    return {};
  }

  std::list<std::pair<float, KeyFrame*>> lScoreAndMatch;

  // Only compare against those keyframes that share enough words
  int maxCommonWords = 0;
  for (auto* pKFi : lKFsSharingWords) {
    if (pKFi->mnLoopWords > maxCommonWords) {
      maxCommonWords = pKFi->mnLoopWords;
    }
  }

  const int minCommonWords = maxCommonWords * 0.8F;

  // Compute similarity score. Retain the matches whose score is higher than minScore
  for (auto* pKFi : lKFsSharingWords) {
    if (pKFi->mnLoopWords > minCommonWords) {
      const float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);

      pKFi->mLoopScore = si;
      if (si >= minScore) {
        lScoreAndMatch.emplace_back(si, pKFi);
      }
    }
  }

  if (lScoreAndMatch.empty()) {
    return {};
  }

  std::list<std::pair<float, KeyFrame*>> lAccScoreAndMatch;
  float                                  bestAccScore = minScore;

  // Lets now accumulate score by covisibility
  for (auto& [score, pKFi] : lScoreAndMatch) {
    std::vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

    float     bestScore = score;
    float     accScore  = score;
    KeyFrame* pBestKF   = pKFi;
    for (auto* pKF2 : vpNeighs) {
      if (pKF2->mnLoopQuery == pKF->mnId && pKF2->mnLoopWords > minCommonWords) {
        accScore += pKF2->mLoopScore;
        if (pKF2->mLoopScore > bestScore) {
          pBestKF   = pKF2;
          bestScore = pKF2->mLoopScore;
        }
      }
    }

    lAccScoreAndMatch.emplace_back(accScore, pBestKF);
    if (accScore > bestAccScore) {
      bestAccScore = accScore;
    }
  }

  // Return all those keyframes with a score higher than 0.75*bestScore
  const float minScoreToRetain = 0.75F * bestAccScore;

  std::set<KeyFrame*>    spAlreadyAddedKF;
  std::vector<KeyFrame*> vpLoopCandidates;
  vpLoopCandidates.reserve(lAccScoreAndMatch.size());

  for (auto& [accScore, pKFi] : lAccScoreAndMatch) {
    if (accScore > minScoreToRetain) {
      if (!spAlreadyAddedKF.contains(pKFi)) {
        vpLoopCandidates.push_back(pKFi);
        spAlreadyAddedKF.insert(pKFi);
      }
    }
  }

  return vpLoopCandidates;
}

void KeyFrameDatabase::DetectCandidates(
  KeyFrame*               pKF,
  float                   minScore,
  std::vector<KeyFrame*>& vpLoopCand,
  std::vector<KeyFrame*>& vpMergeCand
) {
  const std::set<KeyFrame*> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
  std::list<KeyFrame*>      lKFsSharingWordsLoop;
  std::list<KeyFrame*>      lKFsSharingWordsMerge;

  // Search all keyframes that share a word with current keyframes
  // Discard keyframes connected to the query keyframe
  {
    const std::unique_lock<std::mutex> lock(mMutex);

    for (const auto& [wordId, wordValue] : pKF->mBowVec) {
      std::list<KeyFrame*>& lKFs = mvInvertedFile[wordId];

      for (auto* pKFi : lKFs) {
        if (pKFi->GetMap() == pKF->GetMap()) { // For consider a loop candidate it a candidate it
                                               // must be in the same map
          if (pKFi->mnLoopQuery != pKF->mnId) {
            pKFi->mnLoopWords = 0;
            if (!spConnectedKeyFrames.contains(pKFi)) {
              pKFi->mnLoopQuery = pKF->mnId;
              lKFsSharingWordsLoop.push_back(pKFi);
            }
          }
          pKFi->mnLoopWords++;
        } else if (!pKFi->GetMap()->IsBad()) {
          if (pKFi->mnMergeQuery != pKF->mnId) {
            pKFi->mnMergeWords = 0;
            if (!spConnectedKeyFrames.contains(pKFi)) {
              pKFi->mnMergeQuery = pKF->mnId;
              lKFsSharingWordsMerge.push_back(pKFi);
            }
          }
          pKFi->mnMergeWords++;
        }
      }
    }
  }

  if (lKFsSharingWordsLoop.empty() && lKFsSharingWordsMerge.empty()) {
    return;
  }

  if (!lKFsSharingWordsLoop.empty()) {
    std::list<std::pair<float, KeyFrame*>> lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    int maxCommonWords = 0;
    for (auto* pKFi : lKFsSharingWordsLoop) {
      if (pKFi->mnLoopWords > maxCommonWords) {
        maxCommonWords = pKFi->mnLoopWords;
      }
    }

    const int minCommonWords = maxCommonWords * 0.8F;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    for (auto* pKFi : lKFsSharingWordsLoop) {
      if (pKFi->mnLoopWords > minCommonWords) {
        const float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);

        pKFi->mLoopScore = si;
        if (si >= minScore) {
          lScoreAndMatch.emplace_back(si, pKFi);
        }
      }
    }

    if (!lScoreAndMatch.empty()) {
      std::list<std::pair<float, KeyFrame*>> lAccScoreAndMatch;
      float                                  bestAccScore = minScore;

      // Lets now accumulate score by covisibility
      for (auto& [score, pKFi] : lScoreAndMatch) {
        std::vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float     bestScore = score;
        float     accScore  = score;
        KeyFrame* pBestKF   = pKFi;
        for (auto* pKF2 : vpNeighs) {
          if (pKF2->mnLoopQuery == pKF->mnId && pKF2->mnLoopWords > minCommonWords) {
            accScore += pKF2->mLoopScore;
            if (pKF2->mLoopScore > bestScore) {
              pBestKF   = pKF2;
              bestScore = pKF2->mLoopScore;
            }
          }
        }

        lAccScoreAndMatch.emplace_back(accScore, pBestKF);
        if (accScore > bestAccScore) {
          bestAccScore = accScore;
        }
      }

      // Return all those keyframes with a score higher than 0.75*bestScore
      const float minScoreToRetain = 0.75F * bestAccScore;

      std::set<KeyFrame*> spAlreadyAddedKF;
      vpLoopCand.reserve(lAccScoreAndMatch.size());

      for (auto& [accScore, pKFi] : lAccScoreAndMatch) {
        if (accScore > minScoreToRetain) {
          if (!spAlreadyAddedKF.contains(pKFi)) {
            vpLoopCand.push_back(pKFi);
            spAlreadyAddedKF.insert(pKFi);
          }
        }
      }
    }
  }

  if (!lKFsSharingWordsMerge.empty()) {
    std::list<std::pair<float, KeyFrame*>> lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    int maxCommonWords = 0;
    for (auto* pKFi : lKFsSharingWordsMerge) {
      if (pKFi->mnMergeWords > maxCommonWords) {
        maxCommonWords = pKFi->mnMergeWords;
      }
    }

    const int minCommonWords = maxCommonWords * 0.8F;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    for (auto* pKFi : lKFsSharingWordsMerge) {
      if (pKFi->mnMergeWords > minCommonWords) {
        const float si = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);

        pKFi->mMergeScore = si;
        if (si >= minScore) {
          lScoreAndMatch.emplace_back(si, pKFi);
        }
      }
    }

    if (!lScoreAndMatch.empty()) {
      std::list<std::pair<float, KeyFrame*>> lAccScoreAndMatch;
      float                                  bestAccScore = minScore;

      // Lets now accumulate score by covisibility
      for (auto& [score, pKFi] : lScoreAndMatch) {
        std::vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float     bestScore = score;
        float     accScore  = score;
        KeyFrame* pBestKF   = pKFi;
        for (auto* pKF2 : vpNeighs) {
          if (pKF2->mnMergeQuery == pKF->mnId && pKF2->mnMergeWords > minCommonWords) {
            accScore += pKF2->mMergeScore;
            if (pKF2->mMergeScore > bestScore) {
              pBestKF   = pKF2;
              bestScore = pKF2->mMergeScore;
            }
          }
        }

        lAccScoreAndMatch.emplace_back(accScore, pBestKF);
        if (accScore > bestAccScore) {
          bestAccScore = accScore;
        }
      }

      // Return all those keyframes with a score higher than 0.75*bestScore
      const float minScoreToRetain = 0.75F * bestAccScore;

      std::set<KeyFrame*> spAlreadyAddedKF;
      vpMergeCand.reserve(lAccScoreAndMatch.size());

      for (auto& [accScore, pKFi] : lAccScoreAndMatch) {
        if (accScore > minScoreToRetain) {
          if (!spAlreadyAddedKF.contains(pKFi)) {
            vpMergeCand.push_back(pKFi);
            spAlreadyAddedKF.insert(pKFi);
          }
        }
      }
    }
  }

  for (const auto& [wordId, wordValue] : pKF->mBowVec) {
    std::list<KeyFrame*>& lKFs = mvInvertedFile[wordId];

    for (auto* pKFi : lKFs) {
      pKFi->mnLoopQuery  = -1;
      pKFi->mnMergeQuery = -1;
    }
  }
}

void KeyFrameDatabase::DetectBestCandidates(
  KeyFrame*               pKF,
  std::vector<KeyFrame*>& vpLoopCand,
  std::vector<KeyFrame*>& vpMergeCand,
  int                     nMinWords
) {
  std::list<KeyFrame*> lKFsSharingWords;
  std::set<KeyFrame*>  spConnectedKF;

  // Search all keyframes that share a word with current frame
  {
    const std::unique_lock<std::mutex> lock(mMutex);

    spConnectedKF = pKF->GetConnectedKeyFrames();

    for (const auto& [wordId, wordValue] : pKF->mBowVec) {
      std::list<KeyFrame*>& lKFs = mvInvertedFile[wordId];

      for (auto* pKFi : lKFs) {
        if (spConnectedKF.find(pKFi) != spConnectedKF.end()) {
          continue;
        }
        if (pKFi->mnPlaceRecognitionQuery != pKF->mnId) {
          pKFi->mnPlaceRecognitionWords = 0;
          pKFi->mnPlaceRecognitionQuery = pKF->mnId;
          lKFsSharingWords.push_back(pKFi);
        }
        pKFi->mnPlaceRecognitionWords++;
      }
    }
  }
  if (lKFsSharingWords.empty()) {
    return;
  }

  // Only compare against those keyframes that share enough words
  int maxCommonWords = 0;
  for (auto* pKFi : lKFsSharingWords) {
    if (pKFi->mnPlaceRecognitionWords > maxCommonWords) {
      maxCommonWords = pKFi->mnPlaceRecognitionWords;
    }
  }

  int minCommonWords = maxCommonWords * 0.8F;

  if (minCommonWords < nMinWords) {
    minCommonWords = nMinWords;
  }

  std::list<std::pair<float, KeyFrame*>> lScoreAndMatch;

  // Compute similarity score.
  for (auto* pKFi : lKFsSharingWords) {
    if (pKFi->mnPlaceRecognitionWords > minCommonWords) {
      const float si               = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);
      pKFi->mPlaceRecognitionScore = si;
      lScoreAndMatch.emplace_back(si, pKFi);
    }
  }

  if (lScoreAndMatch.empty()) {
    return;
  }

  std::list<std::pair<float, KeyFrame*>> lAccScoreAndMatch;
  float                                  bestAccScore = 0;

  // Lets now accumulate score by covisibility
  for (auto& [score, pKFi] : lScoreAndMatch) {
    std::vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

    float     bestScore = score;
    float     accScore  = bestScore;
    KeyFrame* pBestKF   = pKFi;
    for (auto* pKF2 : vpNeighs) {
      if (pKF2->mnPlaceRecognitionQuery != pKF->mnId) {
        continue;
      }

      accScore += pKF2->mPlaceRecognitionScore;
      if (pKF2->mPlaceRecognitionScore > bestScore) {
        pBestKF   = pKF2;
        bestScore = pKF2->mPlaceRecognitionScore;
      }
    }
    lAccScoreAndMatch.emplace_back(accScore, pBestKF);
    if (accScore > bestAccScore) {
      bestAccScore = accScore;
    }
  }

  // Return all those keyframes with a score higher than 0.75*bestScore
  const float         minScoreToRetain = 0.75F * bestAccScore;
  std::set<KeyFrame*> spAlreadyAddedKF;
  vpLoopCand.reserve(lAccScoreAndMatch.size());
  vpMergeCand.reserve(lAccScoreAndMatch.size());
  for (auto& [si, pKFi] : lAccScoreAndMatch) {
    if (si > minScoreToRetain) {
      if (!spAlreadyAddedKF.contains(pKFi)) {
        if (pKF->GetMap() == pKFi->GetMap()) {
          vpLoopCand.push_back(pKFi);
        } else {
          vpMergeCand.push_back(pKFi);
        }
        spAlreadyAddedKF.insert(pKFi);
      }
    }
  }
}

bool compFirst(const std::pair<float, KeyFrame*>& a, const std::pair<float, KeyFrame*>& b) {
  return a.first > b.first;
}

void KeyFrameDatabase::DetectNBestCandidates(
  KeyFrame*               pKF,
  std::vector<KeyFrame*>& vpLoopCand,
  std::vector<KeyFrame*>& vpMergeCand,
  int                     nNumCandidates
) {
  std::list<KeyFrame*> lKFsSharingWords;
  std::set<KeyFrame*>  spConnectedKF;

  // Search all keyframes that share a word with current frame
  {
    const std::unique_lock<std::mutex> lock(mMutex);

    spConnectedKF = pKF->GetConnectedKeyFrames();

    for (const auto& [wordId, wordValue] : pKF->mBowVec) {
      std::list<KeyFrame*>& lKFs = mvInvertedFile[wordId];

      for (auto* pKFi : lKFs) {
        if (pKFi->mnPlaceRecognitionQuery != pKF->mnId) {
          pKFi->mnPlaceRecognitionWords = 0;
          if (!spConnectedKF.contains(pKFi)) {
            pKFi->mnPlaceRecognitionQuery = pKF->mnId;
            lKFsSharingWords.push_back(pKFi);
          }
        }
        pKFi->mnPlaceRecognitionWords++;
      }
    }
  }
  if (lKFsSharingWords.empty()) {
    return;
  }

  // Only compare against those keyframes that share enough words
  int maxCommonWords = 0;
  for (auto* pKFi : lKFsSharingWords) {
    if (pKFi->mnPlaceRecognitionWords > maxCommonWords) {
      maxCommonWords = pKFi->mnPlaceRecognitionWords;
    }
  }

  const int minCommonWords = maxCommonWords * 0.8F;

  std::list<std::pair<float, KeyFrame*>> lScoreAndMatch;

  // Compute similarity score.
  for (auto* pKFi : lKFsSharingWords) {
    if (pKFi->mnPlaceRecognitionWords > minCommonWords) {
      const float si               = mpVoc->score(pKF->mBowVec, pKFi->mBowVec);
      pKFi->mPlaceRecognitionScore = si;
      lScoreAndMatch.emplace_back(si, pKFi);
    }
  }

  if (lScoreAndMatch.empty()) {
    return;
  }

  std::list<std::pair<float, KeyFrame*>> lAccScoreAndMatch;
  float                                  bestAccScore = 0;

  // Lets now accumulate score by covisibility
  for (auto& [score, pKFi] : lScoreAndMatch) {
    std::vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

    float     bestScore = score;
    float     accScore  = bestScore;
    KeyFrame* pBestKF   = pKFi;
    for (auto* pKF2 : vpNeighs) {
      if (pKF2->mnPlaceRecognitionQuery != pKF->mnId) {
        continue;
      }

      accScore += pKF2->mPlaceRecognitionScore;
      if (pKF2->mPlaceRecognitionScore > bestScore) {
        pBestKF   = pKF2;
        bestScore = pKF2->mPlaceRecognitionScore;
      }
    }
    lAccScoreAndMatch.emplace_back(accScore, pBestKF);
    if (accScore > bestAccScore) {
      bestAccScore = accScore;
    }
  }

  lAccScoreAndMatch.sort(compFirst);

  vpLoopCand.reserve(nNumCandidates);
  vpMergeCand.reserve(nNumCandidates);
  std::set<KeyFrame*> spAlreadyAddedKF;
  std::size_t         i  = 0;
  auto                it = lAccScoreAndMatch.begin();
  while (i < lAccScoreAndMatch.size()
         && (vpLoopCand.size() < static_cast<std::size_t>(nNumCandidates) || vpMergeCand.size() < static_cast<std::size_t>(nNumCandidates))) {
    KeyFrame* pKFi = it->second;
    if (pKFi->isBad()) {
      continue;
    }

    if (!spAlreadyAddedKF.contains(pKFi)) {
      if (pKF->GetMap() == pKFi->GetMap() && vpLoopCand.size() < static_cast<std::size_t>(nNumCandidates)) {
        vpLoopCand.push_back(pKFi);
      } else if(pKF->GetMap() != pKFi->GetMap() && vpMergeCand.size() < static_cast<std::size_t>(nNumCandidates) && !pKFi->GetMap()->IsBad()) {
        vpMergeCand.push_back(pKFi);
      }
      spAlreadyAddedKF.insert(pKFi);
    }
    i++;
    it++;
  }
}

std::vector<KeyFrame*> KeyFrameDatabase::DetectRelocalizationCandidates(Frame* F, Map* pMap) {
  std::list<KeyFrame*> lKFsSharingWords;

  // Search all keyframes that share a word with current frame
  {
    const std::unique_lock<std::mutex> lock(mMutex);

    for (const auto& [wordId, wordValue] : F->mBowVec) {
      std::list<KeyFrame*>& lKFs = mvInvertedFile[wordId];

      for (auto* pKFi : lKFs) {
        if (pKFi->mnRelocQuery != F->mnId) {
          pKFi->mnRelocWords = 0;
          pKFi->mnRelocQuery = F->mnId;
          lKFsSharingWords.push_back(pKFi);
        }
        pKFi->mnRelocWords++;
      }
    }
  }
  if (lKFsSharingWords.empty()) {
    return {};
  }

  // Only compare against those keyframes that share enough words
  int maxCommonWords = 0;
  for (auto* pKFi : lKFsSharingWords) {
    if (pKFi->mnRelocWords > maxCommonWords) {
      maxCommonWords = pKFi->mnRelocWords;
    }
  }

  const int minCommonWords = maxCommonWords * 0.8F;

  std::list<std::pair<float, KeyFrame*>> lScoreAndMatch;

  // Compute similarity score.
  for (auto* pKFi : lKFsSharingWords) {
    if (pKFi->mnRelocWords > minCommonWords) {
      const float si    = mpVoc->score(F->mBowVec, pKFi->mBowVec);
      pKFi->mRelocScore = si;
      lScoreAndMatch.emplace_back(si, pKFi);
    }
  }

  if (lScoreAndMatch.empty()) {
    return {};
  }

  std::list<std::pair<float, KeyFrame*>> lAccScoreAndMatch;
  float                                  bestAccScore = 0;

  // Lets now accumulate score by covisibility
  for (auto& [score, pKFi] : lScoreAndMatch) {
    std::vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

    float     bestScore = score;
    float     accScore  = bestScore;
    KeyFrame* pBestKF   = pKFi;
    for (auto* pKF2 : vpNeighs) {
      if (pKF2->mnRelocQuery != F->mnId) {
        continue;
      }

      accScore += pKF2->mRelocScore;
      if (pKF2->mRelocScore > bestScore) {
        pBestKF   = pKF2;
        bestScore = pKF2->mRelocScore;
      }
    }
    lAccScoreAndMatch.emplace_back(accScore, pBestKF);
    if (accScore > bestAccScore) {
      bestAccScore = accScore;
    }
  }

  // Return all those keyframes with a score higher than 0.75*bestScore
  const float            minScoreToRetain = 0.75F * bestAccScore;
  std::set<KeyFrame*>    spAlreadyAddedKF;
  std::vector<KeyFrame*> vpRelocCandidates;
  vpRelocCandidates.reserve(lAccScoreAndMatch.size());
  for (auto& [si, pKFi] : lAccScoreAndMatch) {
    if (si > minScoreToRetain) {
      if (pKFi->GetMap() != pMap) {
        continue;
      }
      if (!spAlreadyAddedKF.contains(pKFi)) {
        vpRelocCandidates.push_back(pKFi);
        spAlreadyAddedKF.insert(pKFi);
      }
    }
  }

  return vpRelocCandidates;
}

void KeyFrameDatabase::SetORBVocabulary(ORBVocabulary* pORBVoc) {
  ORBVocabulary** ptr = nullptr;
  ptr  = (ORBVocabulary**)(&mpVoc);
  *ptr = pORBVoc;

  mvInvertedFile.clear();
  mvInvertedFile.resize(mpVoc->size());
}

} // namespace ORB_SLAM3
