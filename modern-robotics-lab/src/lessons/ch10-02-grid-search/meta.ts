import type { LessonMeta } from '@/types';

export const meta: LessonMeta = {
  id: 'ch10-02-grid-search',
  chapter: 10,
  order: 2,
  title: '格點法：Dijkstra、A* 與 wavefront（grid search）',
  bookRef: '§10.2.3–10.4',
  summary: '把 C-space 離散成格點圖，逐步播放 Dijkstra、A* 與 wavefront 的展開順序，理解可容許啟發式與最佳性、解析度完備性與格點數爆炸。',
  estMinutes: 30,
  tags: ['A*', 'Dijkstra', 'wavefront', 'resolution complete'],
};
