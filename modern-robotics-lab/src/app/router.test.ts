import { describe, it, expect } from 'vitest';
import { parseRoute, lessonHref } from './router';

describe('parseRoute', () => {
  it('parses home, lesson and chapter routes', () => {
    expect(parseRoute('')).toEqual({ name: 'home' });
    expect(parseRoute('#/')).toEqual({ name: 'home' });
    expect(parseRoute('#/lesson/ch03-02-rotation-matrices')).toEqual({ name: 'lesson', id: 'ch03-02-rotation-matrices' });
    expect(parseRoute('#/chapter/11')).toEqual({ name: 'chapter', chapter: 11 });
    expect(parseRoute('#/nope').name).toBe('notfound');
  });
  it('round-trips lesson hrefs', () => {
    expect(parseRoute(lessonHref('ch00-01-how-to-use'))).toEqual({ name: 'lesson', id: 'ch00-01-how-to-use' });
  });
});
